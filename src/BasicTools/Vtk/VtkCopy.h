/*
 * VtkCopy.h
 *
 *  Created on: Jul 8, 2010
 *      Author: papazov
 */

#ifndef VTKCOPY_H_
#define VTKCOPY_H_

#include <vtkPoints.h>

class VtkCopy
{
public:
	VtkCopy();
	virtual ~VtkCopy();

	inline void copyPoints(vtkPoints* in, vtkPoints* out);
};

//=== inline methods ==============================================================================================

inline void VtkCopy::copyPoints(vtkPoints* in, vtkPoints* out)
{
	out->SetNumberOfPoints(in->GetNumberOfPoints());
	double p[3];

	for ( vtkIdType i = 0 ; i < in->GetNumberOfPoints() ; ++i )
	{
		in->GetPoint(i, p);
		out->SetPoint(i, p);
	}
}

//=================================================================================================================

#endif /* VTKCOPY_H_ */
