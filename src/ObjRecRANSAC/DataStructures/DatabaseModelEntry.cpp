#include "DatabaseModelEntry.h"
#include "../ORRDefines.h"
#include "ORROctree/ORROctreeNodeData.h"
#include <BasicTools/DataStructures/PointSetAdapter.h>
#include <BasicTools/Vtk/VtkMeshSampler.h>
#include <vtkPointLocator.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <map>

using namespace std;


DatabaseModelEntry::DatabaseModelEntry()
{
	// Create own model (will be used by other classes)
	mOwnModel = vtkPolyData::New();
	this->resetInstanceCounter();
	mHighResModel = NULL;
	mNormals = NULL;
	mValidNormal = NULL;
}

DatabaseModelEntry::~DatabaseModelEntry()
{
	mOwnModel->Delete();
	if ( mNormals ) delete[] mNormals;
	if ( mValidNormal ) delete[] mValidNormal;
}

//=============================================================================================================================

bool DatabaseModelEntry::init(vtkPolyData* mesh, UserData* userData, int numberOfPointsPerLayer, int id)
{
#if (defined OBJ_REC_RANSAC_VERBOSE) || (defined OBJ_REC_RANSAC_VERBOSE_1)
	printf("DatabaseModelEntry::%s():\n", __func__);
#endif

	vtkDataArray* mesh_normals = mesh->GetPointData()->GetNormals();
	if ( !mesh_normals )
	{
		fprintf(stderr, "ERROR in 'DatabaseModelEntry::%s()': input mesh has no normals.\n", __func__);
		fflush(stderr);
		return false;
	}
	if ( mOwnModel->GetNumberOfPoints() <= 0 )
	{
		fprintf(stderr, "ERROR in 'DatabaseModelEntry::%s()': the own model is empty - initialize it first!\n", __func__);
		return false;
	}

	// Save some info
	mHighResModel = mesh;
	mUserData = userData;
	mId = id;

#if (defined OBJ_REC_RANSAC_VERBOSE) || (defined OBJ_REC_RANSAC_VERBOSE_1)
	printf("\tsampling ...\n");
#endif

	// Create the own point set
	vtkPoints* tmp = vtkPoints::New(VTK_DOUBLE);
	VtkMeshSampler sampler;
	// Do we have triangles in the 'mesh'
	if ( mesh->GetNumberOfPolys() )
		sampler.sample(mesh, tmp, ORR_NUM_OF_OWN_MODEL_POINTS); // Sample from the mesh surface		
	else
		sampler.sample(mesh->GetPoints(), tmp, ORR_NUM_OF_OWN_MODEL_POINTS); // Get a subset of the vertices

#if (defined OBJ_REC_RANSAC_VERBOSE) || (defined OBJ_REC_RANSAC_VERBOSE_1)
	printf("\tcopying to own model ...\n");
#endif

	// Copy to the own model
	PointSetAdapter psa;
	psa.vtk2PointSet(tmp, &mOwnPointSet);
	tmp->Delete();

	// Cleanup
	if ( mNormals ) delete[] mNormals;
	if ( mValidNormal ) delete[] mValidNormal;

	// Set the normals for the own model
	int numOfOwnPoints = mOwnPointSet.getNumberOfPoints();
	mNormals = new double[3*numOfOwnPoints];
	mValidNormal = new unsigned char[numOfOwnPoints];
	memset(mValidNormal, 1, numOfOwnPoints*sizeof(unsigned char));

	// Some variables
	const double *n, *own_points = mOwnPointSet.getPoints();
	double *normals = mNormals;
	unsigned char *valid_normal = mValidNormal;
	double p[3];

	vtkPointLocator *locator = vtkPointLocator::New();
	  locator->SetDataSet(mesh);
	  locator->BuildLocator();

#if (defined OBJ_REC_RANSAC_VERBOSE) || (defined OBJ_REC_RANSAC_VERBOSE_1)
	printf("\tsetting the normals ...\n");
#endif

	// Set the normals
	for ( int i = 0 ; i < numOfOwnPoints ; ++i, ++valid_normal, normals += 3, own_points += 3 )
	{
		mesh->GetPoint(locator->FindClosestPoint(own_points), p);
		// Now we know for sure that 'p' is inside the own octree
		OctreeNode* full_leaf = mModelOctree.getFullLeaf(p);
		if ( full_leaf == NULL )
		{
			fprintf(stderr, "WARNING in 'DatabaseModelEntry::%s()': NULL leaf! Check if the model octree was built from the same\n"
					"input mesh as the one used to call this method.\n", __func__);
			*valid_normal = 0; // Invalid normal!
			continue;
		}

		// Get the leaf normal
		n = ((ORROctreeNodeData*)full_leaf->getData())->getNormal();
		if ( n == NULL )
		{
			fprintf(stderr, "WARNING in 'DatabaseModelEntry::%s()': NULL leaf normal!\n", __func__);
			*valid_normal = 0; // Invalid normal!
			continue;
		}

		// Copy to the own normals
		vec_copy3(n, normals);
	}

	// Cleanup
	locator->Delete();

#if (defined OBJ_REC_RANSAC_VERBOSE) || (defined OBJ_REC_RANSAC_VERBOSE_1)
	printf("DatabaseModelEntry::%s(): done.\n", __func__);
#endif

	return true;
}

//=============================================================================================================================
