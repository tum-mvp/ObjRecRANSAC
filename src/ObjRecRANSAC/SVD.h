#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>

typedef float FloatType;

/*
 * Small helper class for a 3D vector. 
 * Allows elements to be accessed as x, y, z 
 * Or as v[0], v[1], v[2].
 */
class Vector3
{
public:
	union 
	{
		FloatType v[3];
		struct
		{
			FloatType x, y, z;
		};
	};

	__device__ __host__ FloatType& operator[](const int idx)
	{
		return v[idx];
	}

	__device__ __host__ Vector3() : x(0), y(0), z(0)
	{

	}
	__device__ __host__ Vector3(FloatType x, FloatType y, FloatType z) : x(x), y(y), z(z)
	{

	}
};

/*
 * 3x3 Matrix class. 
 * Allows elements to be accessed as m00, m01, ... 
 * Or as v[0], v[1], ...
 * Elements are stored in row major order
 * m00 = v[0]; m01 = v[1]; ...
 */
struct Matrix3x3 
{
	union 
	{
		FloatType v[9];
		struct
		{
			FloatType m00, m01, m02;
			FloatType m10, m11, m12;
			FloatType m20, m21, m22;
		};
	};

	__device__ __host__ FloatType& operator[](const int idx)
	{
		return v[idx];
	}

	/*
	 *	Identity constructor
	 */
	__device__ __host__ Matrix3x3() :
		m00(1), m01(0), m02(0), 
		m10(0), m11(1), m12(0), 
		m20(0), m21(0), m22(1)

	{

	}

	/*
	 *	set elements constructor
	 */
	__device__ __host__ Matrix3x3(FloatType m00, FloatType m01, FloatType m02, FloatType m10, FloatType m11, FloatType m12, FloatType m20, FloatType m21, FloatType m22) :
		m00(m00), m01(m01), m02(m02), 
		m10(m10), m11(m11), m12(m12), 
		m20(m20), m21(m21), m22(m22)

	{

	}

	/*
	 *	Copy constructor
	 */
	__device__ __host__ Matrix3x3(const Matrix3x3& M) :
	m00(M.m00), m01(M.m01), m02(M.m02), 
		m10(M.m10), m11(M.m11), m12(M.m12), 
		m20(M.m20), m21(M.m21), m22(M.m22)

	{

	}

	/*
	 *	Element-wise minus operator
	 */
	__device__ __host__ Matrix3x3 operator-(Matrix3x3& B) {
		Matrix3x3& A = *this;
		Matrix3x3 C; 

		for(int i=0; i<9; ++i) C[i] = A[i] - B[i];

		return C;
	}

	/*
	 *	Calculate sum of all squared elements
	 */
	__device__ __host__ FloatType SquaredSum() {
		Matrix3x3& A = *this;
		FloatType weight = 0;

		for(int i=0; i<9; ++i) weight += A[i]*A[i];

		return weight;
	}

	/*
	 *	Matrix*Matrix multiplication
	 */
	__device__ __host__ Matrix3x3 operator*(const Matrix3x3& A) const {
		const Matrix3x3& B = *this;
		Matrix3x3 C; 

		C.m00 = A.m00*B.m00 + A.m10*B.m01 + A.m20*B.m02;
		C.m10 = A.m00*B.m10 + A.m10*B.m11 + A.m20*B.m12;
		C.m20 = A.m00*B.m20 + A.m10*B.m21 + A.m20*B.m22;

		C.m01 = A.m01*B.m00 + A.m11*B.m01 + A.m21*B.m02;
		C.m11 = A.m01*B.m10 + A.m11*B.m11 + A.m21*B.m12;
		C.m21 = A.m01*B.m20 + A.m11*B.m21 + A.m21*B.m22;

		C.m02 = A.m02*B.m00 + A.m12*B.m01 + A.m22*B.m02;
		C.m12 = A.m02*B.m10 + A.m12*B.m11 + A.m22*B.m12;
		C.m22 = A.m02*B.m20 + A.m12*B.m21 + A.m22*B.m22;

		return C;
	}

	/*
	 *	Matrix*Vector multiplication
	 */
	__device__ __host__ Vector3 operator*(const Vector3& B) const {
		const Matrix3x3& A = *this;
		Vector3 C; 

		C.x = A.m00*B.x + A.m01*B.y + A.m02*B.z;
		C.y = A.m10*B.x + A.m11*B.y + A.m12*B.z;
		C.z = A.m20*B.x + A.m21*B.y + A.m22*B.z;

		return C;
	}

	/*
	 *	3x3 Determinant
	 */
	__device__ __host__ FloatType Determinant() {
		FloatType out;

		out  = m00*m11*m22;
		out += m01*m12*m20;
		out += m02*m10*m21;
		
		out -= m20*m11*m02;
		out -= m10*m01*m22;
		out -= m00*m21*m12;

		return out;
	}
};


/*
 *	Returns a transposed version of the matrix
 */
__device__ __host__ inline Matrix3x3 Transposed(Matrix3x3 A) {
	Matrix3x3 out;

	out.m00 = A.m00;
	out.m01 = A.m10;
	out.m02 = A.m20;

	out.m10 = A.m01;
	out.m11 = A.m11;
	out.m12 = A.m21;

	out.m20 = A.m02;
	out.m21 = A.m12;
	out.m22 = A.m22;

	return out;
}

/*
 * Quaternion helper class
 * Allows elements to be accessed as x, y, z, w
 * Or as v[0], v[1], v[2], v[3].
 */
struct Quaternion 
{
	union 
	{
		FloatType v[4];
		struct
		{
			FloatType x, y, z, w;
		};
	};

	__device__ __host__ FloatType& operator[](const int idx)
	{
		return v[idx];
	}

	/*
	 *	Identity
	 */
	__device__ __host__ Quaternion() : x(0), y(0), z(0), w(1)
	{

	}

	/*
	 *	Basic Constructor
	 */
	__device__ __host__ Quaternion(FloatType x, FloatType y, FloatType z, FloatType w) : x(x), y(y), z(z), w(w)
	{

	}

	/*
	 *	Quaternion concatenation operator
	 */
	__device__ __host__ Quaternion operator*(const Quaternion& b)
	{
		Quaternion tmp;

		tmp.w = (b.w * w) - (b.x * x) - (b.y * y) - (b.z * z);
		tmp.x = (b.w * x) + (b.x * w) + (b.y * z) - (b.z * y);
		tmp.y = (b.w * y) + (b.y * w) + (b.z * x) - (b.x * z);
		tmp.z = (b.w * z) + (b.z * w) + (b.x * y) - (b.y * x);

		return tmp;
	}

	/**
	 * Normalize the quaternion
	 */
	__device__ __host__ void Normalize()
	{
		FloatType lengthInv = 1.0/sqrt(x*x + y*y + z*z + w*w);
		x *= lengthInv;
		y *= lengthInv;
		z *= lengthInv;
		w *= lengthInv;
	}

	/**
	 * Returns matrix representation of quaternion as a rotation description
	 */
	__device__ __host__ Matrix3x3 GetMatrix()
	{
		Matrix3x3 out;
		
		out.m00 = 1 - 2 * ( y*y + z*z );
		out.m01 =     2 * ( x*y - z*w );
		out.m02 =     2 * ( x*z + y*w );

		out.m10 =     2 * ( x*y + z*w );
		out.m11 = 1 - 2 * ( x*x + z*z );
		out.m12 =     2 * ( y*z - x*w );

		out.m20 =     2 * ( x*z - y*w );
		out.m21 =     2 * ( y*z + x*w );
		out.m22 = 1 - 2 * ( x*x + y*y );
		
		return out;
	}
};

/*
 * Returns Quaternion that diagonalizes A. 
 * A must be symmetric.
 */
__device__ __host__ inline Quaternion Diagonalize(const Matrix3x3& A)
{
	// Diagonal matrix D = Q * A * FloatTyperanspose(Q);  and  A = QFloatType*D*Q
	// FloatTypehe rows of q are the eigenvectors D's diagonal is the eigenvalues
	// As per 'row' convention if float3x3 Q = q.getmatrix(); then v*Q = q*v*conj(q)
	int i;
	Quaternion q = Quaternion(0, 0, 0, 1);
	for(i=0; i<100; i++)
	{
		Matrix3x3 Q  = q.GetMatrix(); // v*Q == q*v*conj(q)
		Matrix3x3 D  = Transposed(Q) * A * Q;  // A = Q^FloatType*D*Q

		Vector3 diag(D.m00, D.m11, D.m22);
		Vector3 offdiag(D.m12, D.m02, D.m01); // elements not on the diagonal
		Vector3 om(fabs(offdiag.x), fabs(offdiag.y), fabs(offdiag.z)); // mag of each offdiag elem
		
		int k = (om.x > om.y && om.x > om.z) ? 0 : (om.y > om.z) ? 1 : 2; // index of largest element of offdiag
		int k1 = (k+1)%3;
		int k2 = (k+2)%3;
		if(offdiag[k]==0.0f) break;  // diagonal already
		
		FloatType thet = (diag[k2]-diag[k1])/(2.0f*offdiag[k]);
		FloatType sgn = (thet > 0.0f) ? 1.0f : -1.0f;
		thet    *= sgn; // make it positive
		FloatType t = sgn /(thet +((thet < 1.E6f)?sqrt(thet*thet+1.0f):thet)) ; // sign(FloatType)/(|FloatType|+sqrt(FloatType^2+1))
		FloatType c = 1.0f/sqrt(t*t+1.0f); //  c= 1/(t^2+1) , t=s/c 
		if(c==1.0f) break;  // no room for improvement - reached machine precision.
		
		Quaternion jr(0,0,0,0); // jacobi rotation for this iteration.
		jr[k] = sgn*sqrt((1.0f-c)/2.0f);  // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)  
		jr[k] *= -1.0f; // since our quat-to-matrix convention was for v*M instead of M*v
		jr.w  = sqrt(1.0f - jr[k]*jr[k]);
		if(jr.w==1.0f) break; // reached limits of floating point precision
		q =  q*jr;  
		q.Normalize();
	} 
	return q;
}

/*
 * Returns matrix containing all eigenvectors
 * Eigenvectors are sorted by the magnitude of their eigenvalues. 
 */
__device__ __host__ inline Matrix3x3 EigenvectorDecomposition(const Matrix3x3& A)
{
	// Calculate eigenvector matrix
	Matrix3x3 Q = Diagonalize(A).GetMatrix();

	// Calculate diagonal matrix. Diagonal entries contain eigenvalues.
	Matrix3x3 D = Transposed(Q) * A * Q;
	Vector3 diag = Vector3(fabs(D.m00), fabs(D.m11), fabs(D.m22));

	// Find index of biggest eigenvalue
	int ev1 = (diag[0] > diag[1] && diag[0] > diag[2]) ? 0 : (diag[1] > diag[2]) ? 1 : 2;
	
	// Put last two eigenvalues into order.
	int ev2 = (ev1+1)%3;
	int ev3 = (ev1+2)%3;
	if(diag[ev3] > diag[ev2])
	{
		int tmp = ev2;
		ev2 = ev3;
		ev3 = tmp;
	}

	// Reorder matrix to start with the eigenvector of the biggest eigenvalue
	Matrix3x3 R;
	R.m00 = Q[0*3+ev1];
	R.m10 = Q[1*3+ev1];
	R.m20 = Q[2*3+ev1];

	R.m01 = Q[0*3+ev2];
	R.m11 = Q[1*3+ev2];
	R.m21 = Q[2*3+ev2];

	R.m02 = Q[0*3+ev3];
	R.m12 = Q[1*3+ev3];
	R.m22 = Q[2*3+ev3];

	return R;
}

/*
 *	Calculates U and V matrix for a singular value decomposition
 *  A = U * S * VFloatType
 *  http://en.wikipedia.org/wiki/Singular_value_decomposition
 */
__device__ __host__ inline void SVD(const Matrix3x3& A, Matrix3x3& U, Matrix3x3& V)
{
	Matrix3x3 R;

	// Create square symmetric matrices
	Matrix3x3 AAFloatType = A * Transposed(A);
	Matrix3x3 AFloatTypeA = Transposed(A) * A;

	// U and V contain the eigenvalues of AAFloatType and AFloatTypeA
	U = EigenvectorDecomposition(AAFloatType);
	V = EigenvectorDecomposition(AFloatTypeA);
	
	// Calculate S diagonal matrix.
	Matrix3x3 S = Transposed(U) * A * V;

	// Singular values have to be positive. 
	// If one of the singular values is negative we have to
	// invert the corresponding column in U or V. I chose U. 
	if(S.m00 < 0) 
	{
		U.m00 = -U.m00;
		U.m10 = -U.m10;
		U.m20 = -U.m20;
	}

	if(S.m11 < 0) 
	{
		U.m01 = -U.m01;
		U.m11 = -U.m11;
		U.m21 = -U.m21;
	}

	if(S.m22 < 0) 
	{
		U.m02 = -U.m02;
		U.m12 = -U.m12;
		U.m22 = -U.m22;
	}
}

/*
 *  Polar decompositon based on singular value decomposition. Q = U*FloatTyperansposed(V)
 */
__device__ __host__ inline Matrix3x3 PolarDecomposition(const Matrix3x3& A)
{
	Matrix3x3 U, V;
    SVD(A, U, V);
    return U*Transposed(V);
}
