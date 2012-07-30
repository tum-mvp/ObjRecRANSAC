#ifndef _TUM_PCA_H_
#define _TUM_PCA_H_

#include <vtkPoints.h>
#include <vtkIdList.h>
#include <list>

using namespace std;

namespace tum
{

class PCA
{
public:
	PCA();
	virtual ~PCA();

	/**
	 * Does %PCA. Computes eigenvectors, eigenvalues and center of mass. Input is the point set 'points'.
	 * The i-th column of eigenvecs belongs to the i-th eigenvalue in eigenvals.
	 * The eigenvalues are sorted in descending order, the eigenvectors have unit length and build an orthonormal
	 * right-hand system. centerOfMass is the center of mass of the points.
	 */
	void doPCA(vtkPoints *points, double eigenvecs[3][3], double eigenvals[3], double centerOfMass[3]);

	/**
	 * Does %PCA. Computes eigenvectors, eigenvalues and center of mass. Input is the point set 'points'.
	 * Only the points with ids in useIds will be used for the computations.
	 * The i-th column of eigenvecs belongs to the i-th eigenvalue in eigenvals.
	 * The eigenvalues are sorted in descending order, the eigenvectors have unit length and build an orthonormal
	 * right-hand system. centerOfMass is the center of mass of the points.
	 */
	void doPCA(vtkPoints *points, list<int>& useIds, double eigenvecs[3][3], double eigenvals[3], double centerOfMass[3]);

	/**
	 * Does %PCA. Computes eigenvectors, eigenvalues and center of mass. Input is a point set 'points'.
	 * Only the points with ids in useIds will be used for the computations.
	 * The i-th column of eigenvecs belongs to the i-th eigenvalue in eigenvals.
	 * The eigenvalues are sorted in descending order, the eigenvectors have unit length and build an orthonormal
	 * right-hand system. centerOfMass is the center of mass of the points.
	 */
	void doPCA(vtkPoints *points, vtkIdList *useIds, double eigenvecs[3][3], double eigenvals[3], double centerOfMass[3]);

	/**
	 * Does %PCA. Computes eigenvectors and eigenvalues. Input is a point set points and the center of mass com
	 * Only the points with ids in useIds will be used for the computations.
	 * The i-th column of eigenvecs belongs to the i-th eigenvalue in eigenvals.
	 * The eigenvalues are sorted in descending order, the eigenvectors have unit length and build an orthonormal
	 * right-hand system.
	 */
	void doPCA(vtkPoints *points, const double com[3], vtkIdList *useIds, double eigenvecs[3][3], double eigenvals[3]);

	/**
	 * Does %PCA. Computes eigenvectors and eigenvalues. Input is a point set points in matrix form with 3 rows
	 * and numOfPoints columns. (points[0][j], points[1][j], points[2][j]) is the j-th point in the set.
	 * The center of mass of the points will be saved in centerOfMass.
	 * The i-th column of eigenvecs belongs to the i-th eigenvalue in eigenvals.
	 * The eigenvalues are sorted in descending order, the eigenvectors have unit length and build an orthonormal
	 * right-hand system.
	 * 
	 * WARNING: the coordinates of points will be altered: they will be centered.
	 */
	void doPCA(double **points, int numOfPoints, double eigenvecs[3][3], double eigenvals[3], double centerOfMass[3]);

	/**
	 * Computes eigenvectors and eigenvalues for the centered points cpts. cpts has to be a matrix
	 * with 3 rows and numOfPoints columns.
	 * 
	 * The eigenvalues are sorted in descending order, the eigenvectors have unit length and build an orthonormal
	 * right-hand system.
	 */
	void eigenComputations(double** cpts, int numOfPoints, double eigenVectors[3][3], double eigenValues[3]);

protected:
	void swap(double *v, int pos1, int pos2);
	void swapCol(double m[3][3], int col1, int col2);
};

}//namespace tum

#endif /*_TUM_PCA_H_*/
