#ifndef VTKWINDOW_H_
#define VTKWINDOW_H_

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProp.h>
#include <vtkInteractorStyle.h>
#include <vtkPointPicker.h>
#include <vtkVolume.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include "VtkLine.h"

class VtkWindow: public vtkCommand
{
public:
	VtkWindow();
	VtkWindow(int posx, int posy, int width, int height);
	virtual ~VtkWindow();

	void printCameraInfo(bool value){ mPrintCamInfo = value;}

	void addToRenderer(vtkProp *obj){ mRenderer->AddActor(obj);}
	void removeFromRenderer(vtkActor *actor){ mRenderer->RemoveActor(actor);}
	void addVolume(vtkVolume* vol){ mRenderer->AddVolume(vol);}
	void render(){ mRenderer->ResetCameraClippingRange(); mRenWin->Render();}
	void vtkMainLoop(){ this->render(); mIRen->Start();}

	void showAxes(){mXAxis->getActor()->VisibilityOn();  mYAxis->getActor()->VisibilityOn();  mZAxis->getActor()->VisibilityOn();}
	void hideAxes(){mXAxis->getActor()->VisibilityOff(); mYAxis->getActor()->VisibilityOff(); mZAxis->getActor()->VisibilityOff();}
	void setAxesLength(double len){mXAxis->setLength(len); mYAxis->setLength(len); mZAxis->setLength(len);}
	void setAxesRadius(double rad){mXAxis->setRadius(rad); mYAxis->setRadius(rad); mZAxis->setRadius(rad);}

	void setBackgroundColor(double r, double g, double b){ this->mRenderer->SetBackground(r, g, b);}
	void setWindowSize(int width, int height){ this->mRenWin->SetSize(width, height);}

	void parallelProjectionOn(){ mRenderer->GetActiveCamera()->ParallelProjectionOn();}
	void perspectiveProjectionOn(){ mRenderer->GetActiveCamera()->ParallelProjectionOff();}

	vtkRenderWindow* getRenderWindow(){ return mRenWin;}
	vtkRenderer* getRenderer(){ return mRenderer;}
	vtkRenderWindowInteractor* getRenderWindowInteractor(){ return mIRen;}

	void rotateCamAboutFocalPointX(double angle);
	void rotateCamAboutFocalPointY(double angle);
	void rotateCamAboutFocalPointZ(double angle);

	void resetView(){mRenderer->ResetCamera(); mRenderer->ResetCameraClippingRange(); mRenWin->Render();}

	// Camera stuff
	void setCameraFocalPoint(double x, double y, double z){ mRenderer->GetActiveCamera()->SetFocalPoint(x, y, z);}
	void setCameraPosition(double x, double y, double z){ mRenderer->GetActiveCamera()->SetPosition(x, y, z);}
	void setCameraViewupVector(double x, double y, double z){ mRenderer->GetActiveCamera()->SetViewUp(x, y, z);}
	void setCameraViewAngle(double angle){ mRenderer->GetActiveCamera()->SetViewAngle(angle);}
	void setCameraViewUp(const double viewup[3]){ mRenderer->GetActiveCamera()->SetViewUp(viewup);}
	void setCameraViewUp(double x, double y, double z){ double up[3] = {x,y,z}; mRenderer->GetActiveCamera()->SetViewUp(up);}

	void print_cam_info();

	/**
	 * Inherited from 'vtkCommand'.
	 */
	void Execute(vtkObject *caller, unsigned long eventId, void *callData);

private:
	void init(int posx, int posy, int width, int height);

protected:
	// A renderer and a render window
	vtkRenderer *mRenderer;
	vtkRenderWindow *mRenWin;
	// An interactor
	vtkRenderWindowInteractor *mIRen;
	vtkInteractorStyle *mIStyle;
	vtkPointPicker *mPicker;

	// Axes
	VtkLine *mXAxis, *mYAxis, *mZAxis;
	bool mPrintCamInfo;
};

#endif /*VTKWINDOW_H_*/
