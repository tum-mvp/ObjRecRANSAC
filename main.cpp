/*
 * main.cpp
 *
 *  Created on: Jun 2, 2011
 *      Author: papazov
 *
 *  This is a modified implementation of the method presented in the paper
 *  (freely available on the web):
 *
 *  Chavdar Papazov and Darius Burschka. An Efficient RANSAC for 3D Object
 *  Recognition in Noisy and Occluded Scenes. In Proceedings of the
 *  10th Asian Conference on Computer Vision (ACCV'10), November 2010.
 *
 *  There is exemplary data (a digitized scene containing a table with
 *  several objects on it) provided with this file such that you can run
 *  and see how the method works.
 *
 */

#include <BasicTools/Vtk/VtkTransform.h>
#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <VtkBasics/VtkWindow.h>
#include <VtkBasics/VtkPolyData.h>
#include <VtkBasics/VtkPoints.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkTransformPolyDataFilter.h>
#include <list>

using namespace std;

#define REMOVE_PLANE
#define LOAD_STANDARD_DATA
//#define LOAD_AACHEN_DATA

//=========================================================================================================================

void loadModels(ObjRecRANSAC& objrec, list<UserData*>& userDataList, list<vtkPolyDataReader*>& readers);
void loadModelsAachen(ObjRecRANSAC& objrec, list<UserData*>& userDataList, list<vtkPolyDataReader*>& readers);
void visualize(list<PointSetShape*>& detectedShapes, vtkPoints* scene, vtkPoints* background);

//=========================================================================================================================

int main()
{
	// Some parameters:
	// 'pairwidth' should be roughly half the extent of the visible object part. This means, for each
	// object point p there should be (at least) one point q (from the same object) such that
	// ||p - q|| <= 'pairwidth'.
	// TRADEOFF: smaller values allow for detection in occluded scenes but lead to more imprecise alignment.
	// Bigger values lead to better alignment but require large visible object parts.
	double pairWidth = 35.0;// in millimeter (since the models in this example are in millimeter - see below)
	// 'voxelsize' is the size of the leafs of the octree, i.e., the "size" of the discretization.
	// TRADEOFF: High values lead to less computation time but ignore object details.
	// Small values allow to better distinguish between objects, but will introduce more holes in the resulting
	// "voxel-surface" (especially for a sparsely sampled scene) and thus will make normal computation unreliable.
	// Processing time, of course, will increase with smaller voxel size.
	double voxelSize = 4.0; // in millimeter

	// Create the main object
	ObjRecRANSAC objrec(pairWidth, voxelSize, 0.5/*leave this one like this*/);
        
	// Some lists
	list<UserData*> userDataList; // Look inside the next function to see how to use this list
	list<vtkPolyDataReader*> readers; // This is just to delete the readers at the end
	// Load the models to the hash table. Look inside this function - it is important since you
	// will want to load your own models
#ifdef LOAD_STANDARD_DATA
	loadModels(objrec, userDataList, readers);
#elif defined LOAD_AACHEN_DATA
	loadModelsAachen(objrec, userDataList, readers);
#else
	loadModels(objrec, userDataList, readers);
#endif

	// Some additional parameters for the recognition
	// The desired success probability for object detection. The higher the value the more samples are
	// needed => the more computation time will expire.
	// TRADEOFF: clear.
	double successProbability = 0.99;
	// The "visibility" is the expected visible object part expressed as fraction of the hole object.
	// For example 0.1 means that 10% of the object surface is visible in the scene.
	// Note that the visibility can not be more than 0.5 since a typical scanning device can not see
	// more than the half of the object.
	// TRADEOFF: smaller values allow for a detection in occluded scenes but also lead to more
	// false positives since object hypotheses with small alignment with the scene will be accepted
	objrec.setVisibility(0.2);
	// The "relative object size" is the expected fraction of the scene points which belong to an object.
	// For example a value of 0.05 means that each object present in the scene will contain at
	// least 5% of all scene points.
	// TRADEOFF: lower values lead to more computation time and to higher success probability.
	objrec.setRelativeObjectSize(0.1);
	// This should equal the number of CPU cores
	objrec.setNumberOfThreads(8);

	// This list will contain the model instances which are detected in the scene. After the object detection has been
	// performed, use the 'getUserData()' method for each 'PointSetShape' in the list in order to know which is the
	// object you are currently considering.
	list<PointSetShape*> detectedShapes;

	// Now the (optional) scene pre-processing takes place followed by the object recognition.

	// Load the scene - for this example from the hard drive. In a real scenario it will come from a range scanner.
	// VERY IMPORTANT ASSUMPTIONS:
	// 1) All points have to have positive z coordinates, i.e., all scene points are lying on one side (the "positive" side)
	//    of the scanning device and
	// 2) the origin of the point cloud has to be "in front" of the points, i.e., the rays passing through the origin have to
	//    meet the "outer" part of the objects surface.
	//
	// If 1) doesn't hold, i.e., your scanner provides you with points with negative z coordinates, do *not* just translate
	// the point cloud such that the points get positive z. If you do that, 2) will not hold. Instead, rotate the cloud by
	// 180° around the x or y axis.
	vtkPolyDataReader* sceneReader = vtkPolyDataReader::New();
#ifdef LOAD_STANDARD_DATA
	  sceneReader->SetFileName("data/table_scene.vtk");
#elif defined LOAD_AACHEN_DATA
	  sceneReader->SetFileName("../data/RWTH-Aachen/scene6.vtk");
#else
	  sceneReader->SetFileName("../data/table_scene.vtk");
#endif
	  sceneReader->Update();
	vtkPoints* scene = sceneReader->GetOutput()->GetPoints();
	printf("Scene has %i point(s).\n", (int)scene->GetNumberOfPoints()); fflush(stdout);

	// This is an optional pre-processing: if all objects are on a table and if that table occupies
	// a significant portion of the scene it would make sense to detect the plane and remove all its
	// points and the points below the plane.
#ifdef REMOVE_PLANE
	// Some parameters for the plane removal
	double planeThickness = 15.0; // Since real data is noisy the plane is not infinitely thin
	double relNumOfPlanePoints = 0.2; // At least 20% of the scene points belong to the plane
	RANSACPlaneDetector planeDetector;
	// Perform the plane detection
	planeDetector.detectPlane(scene, relNumOfPlanePoints, planeThickness);
	// Check the orientation of the detected plane normal
	if ( planeDetector.getPlaneNormal()[2] > 0.0 )
		planeDetector.flipPlaneNormal();

	// Get the points above the plane (the scene) and the ones below it (background)
	scene = vtkPoints::New(VTK_DOUBLE);
	vtkPoints* background = vtkPoints::New(VTK_DOUBLE);
	planeDetector.getPointsAbovePlane(scene, background);
#else
	vtkPoints* background = NULL;
#endif

	// Perform the object recognition. You can call this method arbitrary often (perhaps each time with a new scene).
	// However, do NOT forget to free the memory the pointers in the list 'detectedShapes' are pointing to after EACH call!
	objrec.doRecognition(scene, successProbability, detectedShapes);
	printf("%lf seconds elapsed.\n", objrec.getLastOverallRecognitionTimeSec());

	// Do something with the detected model instances, e.g., visualize them
	visualize(detectedShapes, scene, background);

	// Cleanup
	// Destroy the detected shapes
	for ( list<PointSetShape*>::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
		delete *it;
	// Destroy the 'UserData' objects
	for ( list<UserData*>::iterator it = userDataList.begin() ; it != userDataList.end() ; ++it ) delete *it;
	// Destroy the readers
	for ( list<vtkPolyDataReader*>::iterator it = readers.begin() ; it != readers.end() ; ++it ) (*it)->Delete();
	// Destroy the scene reader
	sceneReader->Delete();

#ifdef REMOVE_PLANE
	background->Delete();
	scene->Delete();
#endif

	return 0;
}

//===================================================================================================================

void loadModelsAachen(ObjRecRANSAC& objrec, list<UserData*>& userDataList, list<vtkPolyDataReader*>& readers)
{
	char fileName[1024];
	// Derive the class 'UserData' if you want to save some specific information
	// about each model. When you load a model in the library you have to pass a 'UserData'-pointer
	// to the method 'addModel()'. If the corresponding model is detected in the scene, you can use
	// the 'UserData'-pointer which is returned by the recognition method, in order to know which
	// model has been detected.
	UserData* userData;

	// FIRST MODEL
	// Create a user object.
	userData = new UserData();
	userData->setLabel("Cup"); // Just set a 'Cup' label
	// Load the model
	sprintf(fileName, "../data/RWTH-Aachen/model.wcup.vtk");
	vtkPolyDataReader* reader1 = vtkPolyDataReader::New();
	reader1->SetFileName(fileName);
	reader1->Update();
	// Add the model to the model library
	objrec.addModel(reader1->GetOutput(), userData);
	// Save the user data and the reader in order to delete them later (outside this function)
	userDataList.push_back(userData);
	readers.push_back(reader1);
}

//===================================================================================================================

void loadModels(ObjRecRANSAC& objrec, list<UserData*>& userDataList, list<vtkPolyDataReader*>& readers)
{
	char fileName[1024];
	// Derive the class 'UserData' if you want to save some specific information
	// about each model. When you load a model in the library you have to pass a 'UserData'-pointer
	// to the method 'addModel()'. If the corresponding model is detected in the scene, you can use
	// the 'UserData'-pointer which is returned by the recognition method, in order to know which
	// model has been detected.
	UserData* userData;

	// FIRST MODEL
	// Create a user object.
	userData = new UserData();
	userData->setLabel("Amicelli"); // Just set an 'Amicelli' label
	// Load the model
	sprintf(fileName, "data/Amicelli_Box.vtk");
	vtkPolyDataReader* reader1 = vtkPolyDataReader::New();
	reader1->SetFileName(fileName);
	reader1->Update();
	// Add the model to the model library
	objrec.addModel(reader1->GetOutput(), userData);
	// Save the user data and the reader in order to delete them later (outside this function)
	userDataList.push_back(userData);
	readers.push_back(reader1);

	// SECOND MODEL
	// Create a user object
	userData = new UserData();
	userData->setLabel("Rusk_Box");
	// Load the model
	sprintf(fileName, "data/Rusk_Box.vtk");
	vtkPolyDataReader* reader2 = vtkPolyDataReader::New();
	reader2->SetFileName(fileName);
	reader2->Update();
	// Add the model to the model library
	objrec.addModel(reader2->GetOutput(), userData);
	// Save the user data and the reader in order to delete them later (outside this function)
	userDataList.push_back(userData);
	readers.push_back(reader2);

	// THIRD MODEL
	// Create a user object
	userData = new UserData();
	userData->setLabel("Soda_Club_Bottle");
	// Load the model
	sprintf(fileName, "data/Soda_Club_Bottle.vtk");
	vtkPolyDataReader* reader3 = vtkPolyDataReader::New();
	reader3->SetFileName(fileName);
	reader3->Update();
	// Add the model to the model library
	objrec.addModel(reader3->GetOutput(), userData);
	// Save the user data and the reader in order to delete them later (outside this function)
	userDataList.push_back(userData);
	readers.push_back(reader3);
}

//=========================================================================================================================

void visualize(list<PointSetShape*>& detectedShapes, vtkPoints* scene, vtkPoints* background)
{
	printf("Visualizing ...\n");

	VtkWindow vtkwin(0, 0, 1000, 800);
	  vtkwin.setCameraPosition(131.220071, -240.302073, -162.992888);
	  vtkwin.setCameraFocalPoint(-48.026838, -54.679381, 787.833180);
	  vtkwin.setCameraViewUp(-0.044383, 0.978898, -0.199470);
	  vtkwin.setCameraViewAngle(30.000000);
	  vtkwin.setWindowSize(1000, 800);

	list<VtkPolyData*> transformedModelList;

	// Visualize the detected objects (important to look inside this loop)
	for ( list<PointSetShape*>::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
	{
		PointSetShape* shape = (*it);
		// Which object do we have (and what confidence in the recognition result)
		if ( shape->getUserData() )
			printf("\t%s, confidence: %lf\n", shape->getUserData()->getLabel(), shape->getConfidence());

		// Allocate memory for a homogeneous matrix
		double **mat4x4 = mat_alloc(4, 4);
		// Get the estimated rigid transform
		shape->getHomogeneousRigidTransform(mat4x4);

		// Transform the model instance using the estimated rigid transform
		vtkTransformPolyDataFilter *transformer = vtkTransformPolyDataFilter::New();
		  transformer->SetInput(shape->getHighResModel());
		  VtkTransform::mat4x4ToTransformer((const double**)mat4x4, transformer);

		// Visualize the transformed model
		VtkPolyData* transformedModel = new VtkPolyData(transformer->GetOutput());
		  transformedModel->setColor(1.0, 0.55, 0.05);
		  vtkwin.addToRenderer(transformedModel->getActor());
		  // Save in a list in order to delete outside this loop
		  transformedModelList.push_back(transformedModel);

		// Cleanup
		mat_dealloc(mat4x4, 4);
		transformer->Delete();
	}

	// Visualize the scene
	VtkPoints scenePoints(scene);
	  scenePoints.selfAdjustPointRadius();
	  scenePoints.setColor(0.1, 0.5, 1.0);
	  vtkwin.addToRenderer(scenePoints.getActor());

	// Visualize the background
	VtkPoints* backgroundPoints = NULL;
	if ( background )
	{
            backgroundPoints = new VtkPoints(background);
            backgroundPoints->selfAdjustPointRadius();
            backgroundPoints->setColor(0.8, 0.8, 0.8);
            vtkwin.addToRenderer(backgroundPoints->getActor());
	}

	// The main vtk loop
	vtkwin.vtkMainLoop();

	// Cleanup
	for ( list<VtkPolyData*>::iterator it = transformedModelList.begin() ; it != transformedModelList.end() ; ++it )
		delete *it;
	if ( backgroundPoints )
		delete backgroundPoints;
}

//=========================================================================================================================
