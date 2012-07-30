#include "Matrix.h"
#include "Vector.h"
#include <cmath>

using namespace tum;


Matrix::Matrix()
{
	mRows = mCols = 0;
	m = NULL;
}

Matrix::Matrix(int rows, int cols)
{
	m = NULL;
	this->alloc(rows, cols);
}

Matrix::~Matrix()
{
	this->dealloc();
}

//==================================================================================================================

void Matrix::alloc(int rows, int cols)
{
	this->dealloc();

	mRows = rows;
	mCols = cols;
	m = new double*[rows];
	for ( int i = 0 ; i < rows ; ++i )
		m[i] = new double[cols];
}

//==================================================================================================================

void Matrix::dealloc()
{
	if ( m == NULL )
		return;

	for ( int i = 0 ; i < mRows ; ++i )
		delete[] m[i];
	delete[] m;

	m = NULL;
	mRows = mCols = 0;
}

//==================================================================================================================

void Matrix::print(FILE* file, const char* label)
{
	if ( label )
		fprintf(file, "--- %s ---\n", label);
	for ( int i = 0 ; i < mRows ; ++i )
	{
		for ( int j = 0 ; j < mCols ; ++j )
			fprintf(file, " %lf ", m[i][j]);
		fprintf(file, "\n");
	}
}

//==================================================================================================================

void Matrix::printAsInt(FILE* file, const char* label)
{
	if ( label )
		fprintf(file, "--- %s ---\n", label);
	for ( int i = 0 ; i < mRows ; ++i )
	{
		for ( int j = 0 ; j < mCols ; ++j )
			fprintf(file, " %i ", (int)(m[i][j]+0.5));
		fprintf(file, "\n");
	}

}

//==================================================================================================================

void Matrix::print3x3(FILE* file, double mat[3][3], char* label)
{
	if ( label )
		fprintf(file, "=== %s ===\n", label);
	for ( int i = 0 ; i < 3 ; ++i )
	{
		for ( int j = 0 ; j < 3 ; ++j )
			fprintf(file, " %.9lf ", mat[i][j]);
		fprintf(file, "\n");
	}
}

//==================================================================================================================

void Matrix::print(FILE* file, const double** mat, int width, int height, char* label)
{
	if ( label )
		fprintf(file, "--- %s ---\n", label);
	for ( int i = 0 ; i < height ; ++i )
	{
		for ( int j = 0 ; j < width ; ++j )
			fprintf(file, " %lf ", mat[i][j]);
		fprintf(file, "\n");
	}

}

//==================================================================================================================

bool Matrix::loadFromFile4x4(double** mat4x4, const char* filename)
{
	FILE* fp = fopen(filename, "r");
	if ( !fp )
	{
		fprintf(stderr, "ERROR in 'Matrix::%s()': can not open file '%s'\n", __func__, filename);
		return false;
	}

	// Read the four lines of the file
	fscanf(fp,
			"%lf %lf %lf %lf\n"
			"%lf %lf %lf %lf\n"
			"%lf %lf %lf %lf\n"
			"%lf %lf %lf %lf\n",
			&mat4x4[0][0], &mat4x4[0][1], &mat4x4[0][2], &mat4x4[0][3],
			&mat4x4[1][0], &mat4x4[1][1], &mat4x4[1][2], &mat4x4[1][3],
			&mat4x4[2][0], &mat4x4[2][1], &mat4x4[2][2], &mat4x4[2][3],
			&mat4x4[3][0], &mat4x4[3][1], &mat4x4[3][2], &mat4x4[3][3]);
	fclose(fp);

	return true;
}

//==================================================================================================================

bool Matrix::writeToFile4x4(const double** mat4x4, const char* filename)
{
	FILE* fp = fopen(filename, "w");
	if ( !fp )
	{
		fprintf(stderr, "ERROR in 'Matrix::%s()': can not open file '%s'\n", __func__, filename);
		return false;
	}

	// Read the four lines of the file
	fprintf(fp,
			"%lf %lf %lf %lf\n"
			"%lf %lf %lf %lf\n"
			"%lf %lf %lf %lf\n"
			"%lf %lf %lf %lf\n",
			mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], mat4x4[0][3],
			mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], mat4x4[1][3],
			mat4x4[2][0], mat4x4[2][1], mat4x4[2][2], mat4x4[2][3],
			mat4x4[3][0], mat4x4[3][1], mat4x4[3][2], mat4x4[3][3]);
	fclose(fp);

	return true;
}

//==================================================================================================================

void Matrix::rotateAboutLine3(const double linePoint[3], const double lineAxis[3], double rad, double p[3])
{
	double v[3] = {p[0]-linePoint[0], p[1]-linePoint[1], p[2]-linePoint[2]};
	double s = Vector::dot3(v, lineAxis);
	double w[3] = {s*lineAxis[0], s*lineAxis[1], s*lineAxis[2]};
	// Compute the x-axis
	double y[3], x[3] = {v[0]-w[0], v[1]-w[1], v[2]-w[2]};
	double len = Vector::length3(x);
	Vector::normalize3(x);
	// Compute the y-axis
	Vector::cross3(lineAxis, x, y);

	// 'frame' = [x, y, lineAxis]
	double frame[3][3];
	frame[0][0] = x[0]; frame[0][1] = y[0]; frame[0][2] = lineAxis[0];
	frame[1][0] = x[1]; frame[1][1] = y[1]; frame[1][2] = lineAxis[1];
	frame[2][0] = x[2]; frame[2][1] = y[2]; frame[2][2] = lineAxis[2];

	// Rotate the point in canonical coordinates 
	double pCanRot[3] = {len*cos(rad), len*sin(rad), 0.0};
	// Transform the rotated point from the canonical coordinate system to 'frame'
	Matrix::mult3x3(frame, pCanRot, p);
	p[0] += linePoint[0] + w[0];
	p[1] += linePoint[1] + w[1];
	p[2] += linePoint[2] + w[2];
}

//==================================================================================================================
