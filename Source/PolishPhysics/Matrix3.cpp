#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Matrix3::Matrix3(Precision a, Precision b, Precision c, Precision d, Precision e, Precision f, Precision g, Precision h, Precision i)
{
	mData[0] = a;
	mData[1] = b;
	mData[2] = c;
	mData[3] = d;
	mData[4] = e;
	mData[5] = f;
	mData[6] = g;
	mData[7] = h;
	mData[8] = i;
}

Vector3 Matrix3::operator *(const Vector3& vector) const
{
	return Vector3(vector.X*mData[0] + vector.Y*mData[1] + vector.Z*mData[2],
		vector.X*mData[3] + vector.Y*mData[4] + vector.Z*mData[5],
		vector.X*mData[6] + vector.Y*mData[7] + vector.Z*mData[8]);
}

Vector3 Matrix3::Transform(const Vector3& vector) const
{
	return (*this) * vector;
}

Vector3 Matrix3::TransformTranspose(const Vector3& vector) const
{
	return Vector3(
		vector.X * mData[0] + vector.Y * mData[3] + vector.Z * mData[6],
		vector.X * mData[1] + vector.Y * mData[4] + vector.Z * mData[7],
		vector.X * mData[2] + vector.Y * mData[5] + vector.Z * mData[8]
	);
}

Matrix3 Matrix3::operator*(const Matrix3& other) const
{
	return Matrix3(
		mData[0] * other.mData[0] + mData[1] * other.mData[3] + mData[2] * other.mData[6],
		mData[0] * other.mData[1] + mData[1] * other.mData[4] + mData[2] * other.mData[7],
		mData[0] * other.mData[2] + mData[1] * other.mData[5] + mData[2] * other.mData[8],

		mData[3] * other.mData[0] + mData[4] * other.mData[3] + mData[5] * other.mData[6],
		mData[3] * other.mData[1] + mData[4] * other.mData[4] + mData[5] * other.mData[7],
		mData[3] * other.mData[2] + mData[4] * other.mData[5] + mData[5] * other.mData[8],

		mData[6] * other.mData[0] + mData[7] * other.mData[3] + mData[8] * other.mData[6],
		mData[6] * other.mData[1] + mData[7] * other.mData[4] + mData[8] * other.mData[7],
		mData[6] * other.mData[2] + mData[7] * other.mData[5] + mData[8] * other.mData[8]);
}

void Matrix3::operator*=(const Matrix3& other)
{
	Precision t1;
	Precision t2;
	Precision t3;

	t1 = mData[0] * other.mData[0] + mData[1] * other.mData[3] + mData[2] + other.mData[6];
	t2 = mData[0] * other.mData[1] + mData[1] * other.mData[4] + mData[2] + other.mData[7];
	t3 = mData[0] * other.mData[2] + mData[1] * other.mData[5] + mData[2] + other.mData[8];

	mData[0] = t1;
	mData[1] = t2;	
	mData[2] = t3;

	t1 = mData[3] * other.mData[0] + mData[4] * other.mData[3] + mData[5] + other.mData[6];
	t2 = mData[3] * other.mData[1] + mData[4] * other.mData[4] + mData[5] + other.mData[7];
	t3 = mData[3] * other.mData[2] + mData[4] * other.mData[5] + mData[5] + other.mData[8];

	mData[3] = t1;
	mData[4] = t2;
	mData[5] = t3;

	t1 = mData[6] * other.mData[0] + mData[7] * other.mData[3] + mData[8] + other.mData[6];
	t2 = mData[6] * other.mData[1] + mData[7] * other.mData[4] + mData[8] + other.mData[7];
	t3 = mData[6] * other.mData[2] + mData[7] * other.mData[5] + mData[8] + other.mData[8];

	mData[6] = t1;
	mData[7] = t2;
	mData[8] = t3;
}

void Matrix3::SetInverse(const Matrix3& other)
{
	Precision t1 = other.mData[0] * other.mData[4];
	Precision t2 = other.mData[0] * other.mData[5];
	Precision t3 = other.mData[1] * other.mData[3];
	Precision t4 = other.mData[2] * other.mData[3];
	Precision t5 = other.mData[1] * other.mData[6];
	Precision t6 = other.mData[2] * other.mData[6];

	Precision det = t1 * other.mData[8] - t2 * other.mData[7] - t3 * other.mData[8] +
		t4 * other.mData[7] + t5 * other.mData[5] - t6 * other.mData[4];

	if (det != static_cast<Precision>(0.0f))
	{
		Precision inverseDet = static_cast<Precision> (1.0f / det);

		mData[0] = (other.mData[4] * other.mData[8] - other.mData[5] * other.mData[7]) * inverseDet;
		mData[1] = -(other.mData[1] * other.mData[8] - other.mData[2] * other.mData[7]) * inverseDet;
		mData[2] = (other.mData[1] * other.mData[5] - other.mData[2] * other.mData[4]) * inverseDet;
		mData[3] = -(other.mData[3] * other.mData[8] - other.mData[5] * other.mData[6]) * inverseDet;
		mData[4] = (other.mData[0] * other.mData[8] - t6) * inverseDet;
		mData[5] = -(t2 - t4) * inverseDet;
		mData[6] = (other.mData[3] * other.mData[7] - other.mData[4] * other.mData[6]) * inverseDet;
		mData[7] = -(other.mData[0] * other.mData[7] - t5) * inverseDet;
		mData[8] = (t1 - t3) * inverseDet;

	}
}

Matrix3 Matrix3::Inverse() const
{
	Matrix3 result;
	result.SetInverse(*this);

	return result;
}

void Matrix3::Invert()
{
	SetInverse(*this);
}

void Matrix3::SetTranspose(const Matrix3& other)
{
	mData[0] = other.mData[0];
	mData[1] = other.mData[3];
	mData[2] = other.mData[6];
	mData[3] = other.mData[1];
	mData[4] = other.mData[4];
	mData[5] = other.mData[7];
	mData[6] = other.mData[2];
	mData[7] = other.mData[5];
	mData[8] = other.mData[8];
}

Matrix3 Matrix3::Transpose() const
{
	Matrix3 result;
	result.SetTranspose(*this);

	return result;
}

void Matrix3::SetOrientation(const Quaternion& quaternion)
{
	mData[0] = 1 - (2 * quaternion.j * quaternion.j + 2 * quaternion.k * quaternion.k);
	mData[1] = 2 * quaternion.i * quaternion.j + 2 * quaternion.k * quaternion.r;
	mData[2] = 2 * quaternion.i * quaternion.k - 2 * quaternion.j * quaternion.r;
	mData[3] = 2 * quaternion.i * quaternion.j - 2 * quaternion.k * quaternion.r;
	mData[4] = 1 - (2 * quaternion.i * quaternion.i + 2 * quaternion.k * quaternion.k);
	mData[5] = 2 * quaternion.j * quaternion.k + 2 * quaternion.i * quaternion.r;
	mData[6] = 2 * quaternion.i * quaternion.k + 2 * quaternion.j * quaternion.r;
	mData[7] = 2 * quaternion.j * quaternion.k - 2 * quaternion.i * quaternion.r;
	mData[8] = 1 - (2 * quaternion.i * quaternion.i + 2 * quaternion.j * quaternion.j);
}

void Matrix3::SetComponentsFromVectors(const Vector3& one, const Vector3& two, const Vector3& three)
{
	mData[0] = one.X;
	mData[1] = two.X;
	mData[2] = three.X;
	mData[3] = one.Y;
	mData[4] = two.Y;
	mData[5] = three.Y;
	mData[6] = one.Z;
	mData[7] = two.Z;
	mData[8] = three.Z;
}

void Matrix3::SetInertiaTensorCoeffs(Precision ix, Precision iy, Precision iz, Precision ixy /* = 0 */, Precision ixz /* = 0 */, Precision iyz /* = 0 */)
{
	
		mData[0] = ix;
		mData[1] = mData[3] = -ixy;
		mData[2] = mData[6] = -ixz;
		mData[4] = iy;
		mData[5] = mData[7] = -iyz;
		mData[8] = iz;
}