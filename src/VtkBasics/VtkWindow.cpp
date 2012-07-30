#include "VtkWindow.h"
#include <vtkIndent.h>
#include <vtkProperty.h>
#include <vtkMatrix4x4.h>
#include <vtkRendererCollection.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <cmath>


VtkWindow::VtkWindow(int posx, int posy, int width, int height)
{
	this->init(posx, posy, width, height);
}

VtkWindow::VtkWindow()
{
	this->init(0, 0, 800, 600);
}

void VtkWindow::init(int posx, int posy, int width, int height)
{
	mPrintCamInfo = true;

	this->mRenderer = vtkRenderer::New();
	this->mRenWin = vtkRenderWindow::New();
	this->mIRen = vtkRenderWindowInteractor::New();
	this->mIStyle = vtkInteractorStyleTrackballCamera::New();
//	this->mIStyle = vtkInteractorStyleTerrain::New();
	this->mPicker = vtkPointPicker::New();

	this->mRenWin->SetPosition(posx, posy);
	this->mRenWin->SetSize(width, height);
	this->mRenWin->AddRenderer(mRenderer);
	this->mIRen->SetRenderWindow(mRenWin);
	this->mIRen->SetInteractorStyle(mIStyle);
	this->mIRen->SetPicker(mPicker);
	this->mRenderer->SetBackground(1,1,1);

	// Register yourself as an observer by mIRen.
	mIRen->AddObserver(vtkCommand::KeyReleaseEvent, this, 0.0/*priority*/);

	mXAxis = new VtkLine(0,0,0, 1,0,0);
	mYAxis = new VtkLine(0,0,0, 0,1,0);
	mZAxis = new VtkLine(0,0,0, 0,0,1);

	mXAxis->setRadius(0.003);
	mYAxis->setRadius(0.003);
	mZAxis->setRadius(0.003);

	mXAxis->getActor()->VisibilityOff();
	mYAxis->getActor()->VisibilityOff();
	mZAxis->getActor()->VisibilityOff();

	mXAxis->getActor()->GetProperty()->SetColor(255,0,0);
	mYAxis->getActor()->GetProperty()->SetColor(0,255,0);
	mZAxis->getActor()->GetProperty()->SetColor(0,0,255);

	mRenderer->AddActor(mXAxis->getActor());
	mRenderer->AddActor(mYAxis->getActor());
	mRenderer->AddActor(mZAxis->getActor());
}

VtkWindow::~VtkWindow()
{
	this->mRenderer->Delete();
	this->mRenWin->Delete();
	this->mIRen->Delete();
	this->mIStyle->Delete();
	this->mPicker->Delete();

	delete mXAxis;
	delete mYAxis;
	delete mZAxis;
}

//=========================================================================================================================

void VtkWindow::rotateCamAboutFocalPointX(double angle)
{
	double y, z, pos[3], fp[3];
	vtkCamera *cam = mRenderer->GetActiveCamera();
	cam->GetPosition(pos);
	cam->GetFocalPoint(fp);
	// Translate y and z to the origin
	pos[1] -= fp[1];
	pos[2] -= fp[2];
	// Rotate about the origin and the x-axis
	y = pos[1]*cos(angle) - pos[2]*sin(angle);
	z = pos[1]*sin(angle) + pos[2]*cos(angle);
	pos[1] = y;
	pos[2] = z;
	// Translate back
	pos[1] += fp[1];
	pos[2] += fp[2];
	cam->SetPosition(pos);
}

//=========================================================================================================================

void VtkWindow::rotateCamAboutFocalPointY(double angle)
{
	double x, z, pos[3], fp[3];
	vtkCamera *cam = mRenderer->GetActiveCamera();
	cam->GetPosition(pos);
	cam->GetFocalPoint(fp);
	// Translate x and z to the origin
	pos[0] -= fp[0];
	pos[2] -= fp[2];
	// Rotate about the origin and the y-axis
	x =  pos[0]*cos(angle) + pos[2]*sin(angle);
	z = -pos[0]*sin(angle) + pos[2]*cos(angle);
	pos[0] = x;
	pos[2] = z;
	// Translate back
	pos[0] += fp[0];
	pos[2] += fp[2];
	cam->SetPosition(pos);
}

//=========================================================================================================================

void VtkWindow::rotateCamAboutFocalPointZ(double angle)
{
	double x, y, pos[3], fp[3];
	vtkCamera *cam = mRenderer->GetActiveCamera();
	cam->GetPosition(pos);
	cam->GetFocalPoint(fp);
	// Translate x and y to the origin
	pos[0] -= fp[0];
	pos[1] -= fp[1];
	// Rotate about the origin
	x = pos[0]*cos(angle) - pos[1]*sin(angle);
	y = pos[0]*sin(angle) + pos[1]*cos(angle);
	pos[0] = x;
	pos[1] = y;
	// Translate back
	pos[0] += fp[0];
	pos[1] += fp[1];
	cam->SetPosition(pos);
}

//=========================================================================================================================

void VtkWindow::Execute(vtkObject *caller, unsigned long eventId, void *callData)
{
	vtkRenderWindowInteractor *renwi;
	renwi = dynamic_cast<vtkRenderWindowInteractor*>(caller);

	if ( !renwi )
		return;

	switch ( eventId )
	{
		case vtkCommand::KeyReleaseEvent:
		{
			double campos[3], fpos[3], diff[3], dist = 1.0;
			int signx = 1, signy = 1, signz = 1;
			vtkCamera* cam = NULL;

			switch ( renwi->GetKeyCode() )
			{
				// Do some preparation which have to be done for all x, X, y, Y, z, Z release key events.
				case 'x':
				case 'X':
				case 'y':
				case 'Y':
				case 'z':
				case 'Z':
				{
					vtkRendererCollection* renderers = renwi->GetRenderWindow()->GetRenderers();
					vtkRenderer* renderer = renderers->GetFirstRenderer();
					cam = renderer->GetActiveCamera();
					cam->GetPosition(campos);
					cam->GetFocalPoint(fpos);
					diff[0] = campos[0]-fpos[0]; diff[1] = campos[1]-fpos[1]; diff[2] = campos[2]-fpos[2];
					if ( diff[0] < 0 ) signx = -1;
					if ( diff[1] < 0 ) signy = -1;
					if ( diff[2] < 0 ) signz = -1;
					// Compute and save the distance between the cam position and the focal point
					dist = sqrt(pow(campos[0]-fpos[0],2) + pow(campos[1]-fpos[1],2) + pow(campos[2]-fpos[2],2));
					break;
				}

				case 'p':
				case 'P':
				{
					if ( !mPrintCamInfo )
						break;

					double viewup[3];
					cam = mRenderer->GetActiveCamera();
					cam->GetPosition(campos);
					cam->GetFocalPoint(fpos);
					cam->GetViewUp(viewup);
					int *size = mRenWin->GetSize();

					printf("\nmVtkWin.setCameraPosition(%lf, %lf, %lf);\n"
							"mVtkWin.setCameraFocalPoint(%lf, %lf, %lf);\n"
							"mVtkWin.setCameraViewUp(%lf, %lf, %lf);\n"
							"mVtkWin.setCameraViewAngle(%lf);\n"
							"mVtkWin.setWindowSize(%i, %i);\n",
					       campos[0], campos[1], campos[2], fpos[0], fpos[1], fpos[2],
					       viewup[0], viewup[1], viewup[2], 
					       cam->GetViewAngle(), size[0], size[1]);
					break;
				}
			}
			// Do a second switch for the x, y, ... specific task(s).
			switch ( renwi->GetKeyCode() )
			{
				case 'x':
				case 'X':
					fpos[0] += dist*signx;
					cam->SetPosition(fpos);
					renwi->Render();
					break;
				case 'y':
				case 'Y':
					fpos[1] += dist*signy;
					cam->SetPosition(fpos);
					renwi->Render();
					break;
				case 'z':
				case 'Z':
					fpos[2] += dist*signz;
					cam->SetPosition(fpos);
					renwi->Render();
					break;

				case 27:
					renwi->TerminateApp();
					break;
			}
		}
		default:
			break;
	}
}

//=========================================================================================================================

void VtkWindow::print_cam_info()
{
	double campos[3], viewup[3], fpos[3];
	vtkCamera* cam = mRenderer->GetActiveCamera();
	cam->GetPosition(campos);
	cam->GetFocalPoint(fpos);
	cam->GetViewUp(viewup);
	int *size = mRenWin->GetSize();

	printf("\nCamera position = {%lf, %lf, %lf};\n"
			"Focal point      = {%lf, %lf, %lf};\n"
			"Viewup vector    = {%lf, %lf, %lf};\n"
			"View angle       = %lf;\n"
			"Viewport size    = {%i, %i};\n",
		   campos[0], campos[1], campos[2], fpos[0], fpos[1], fpos[2],
		   viewup[0], viewup[1], viewup[2],
		   cam->GetViewAngle(), size[0], size[1]);
	fflush(stdout);
}

//=========================================================================================================================
