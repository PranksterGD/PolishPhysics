#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Matrix3::Matrix3(Precision a, Precision b, Precision c, Precision d, Precision e, Precision f, Precision g, Precision h, Precision i)
{
	mmData[0] = a;
	mmData[1] = b;
	mmData[2] = c;
	mmData[3] = d;
	mmData[4] = e;
	mmData[5] = f;
	mmData[6] = g;
	mmData[7] = h;
	mmData[8] = i;
}

Vector3 Matrix3::operator *(const Vector3& vector) const
{
	return Vector3(vector.X*mmData[0] + vector.Y*mmData[1] + vector.Z*mmData[2],
		vector.X*mmData[3] + vector.Y*mmData[4] + vector.Z*mmData[5],
		vector.X*mmData[6] + vector.Y*mmData[7] + vector.Z*mmData[8]);
}

Vector3 Matrix3::Transform(const Vector3& vector) const
{
	return (*this) * vector;
}

Matrix3 Matrix3::operator*(const Matrix3& other) const
{
	return Matrix3(
		mmData[0] * other.mmData[0] + mmData[1] * other.mmData[3] + mmData[2] * other.mmData[6],
		mmData[0] * other.mmData[1] + mmData[1] * other.mmData[4] + mmData[2] * other.mmData[7],
		mmData[0] * other.mmData[2] + mmData[1] * other.mmData[5] + mmData[2] * other.mmData[8],

		mmData[3] * other.mmData[0] + mmData[4] * other.mmData[3] + mmData[5] * other.mmData[6],
		mmData[3] * other.mmData[1] + mmData[4] * other.mmData[4] + mmData[5] * other.mmData[7],
		mmData[3] * other.mmData[2] + mmData[4] * other.mmData[5] + mmData[5] * other.mmData[8],

		mmData[6] * other.mmData[0] + mmData[7] * other.mmData[3] + mmData[8] * other.mmData[6],
		mmData[6] * other.mmData[1] + mmData[7] * other.mmData[4] + mmData[8] * other.mmData[7],
		mmData[6] * other.mmData[2] + mmData[7] * other.mmData[5] + mmData[8] * other.mmData[8]);
}

void Matrix3::operator*=(const Matrix3& other)
{
	Precision t1;
	Precision t2;
	Precision t3;

	t1 = mmData[0] * other.mmData[0] + mmData[1] * other.mmData[3] + mmData[2] + other.mmData[6];
	t2 = mmData[0] * other.mmData[1] + mmData[1] * other.mmData[4] + mmData[2] + other.mmData[7];
	t3 = mmData[0] * other.mmData[2] + mmData[1] * other.mmData[5] + mmData[2] + other.mmData[8];

	mmData[0] = t1;
	mmData[1] = t2;	
	mmData[2] = t3;

	t1 = mmData[3] * other.mmData[0] + mmData[4] * other.mmData[3] + mmData[5] + other.mmData[6];
	t2 = mmData[3] * other.mmData[1] + mmData[4] * other.mmData[4] + mmData[5] + other.mmData[7];
	t3 = mmData[3] * other.mmData[2] + mmData[4] * other.mmData[5] + mmData[5] + other.mmData[8];

	mmData[3] = t1;
	mmData[4] = t2;
	mmData[5] = t3;

	t1 = mmData[6] * other.mmData[0] + mmData[7] * other.mmData[3] + mmData[8] + other.mmData[6];
	t2 = mmData[6] * other.mmData[1] + mmData[7] * other.mmData[4] + mmData[8] + other.mmData[7];
	t3 = mmData[6] * other.mmData[2] + mmData[7] * other.mmData[5] + mmData[8] + other.mmData[8];

	mmData[6] = t1;
	mmData[7] = t2;
	mmData[8] = t3;
}

void Matrix3::SetInverse(const Matrix3& other)
{
	Precision t1 = other.mmData[0] * other.mmData[4];
	Precision t2 = other.mmData[0] * other.mmData[5];
	Precision t3 = other.mmData[1] * other.mmData[3];
	Precision t4 = other.mmData[2] * other.mmData[3];
	Precision t5 = other.mmData[1] * other.mmData[6];
	Precision t6 = other.mmData[2] * other.mmData[6];

	Precision det = t1 * other.mmData[8] - t2 * other.mmData[7] - t3 * other.mmData[8] +
		t4 * other.mmData[7] + t5 * other.mmData[5] - t6 * other.mmData[4];

	if (det != static_cast<Precision>(0.0f))
	{
		Precision inverseDet = static_cast<Precision> (1.0f / det);

		mmData[0] = (other.mmData[4] * other.mmData[8] - other.mmData[5] * other.mmData[7]) * inverseDet;
		mmData[1] = -(other.mmData[1] * other.mmData[8] - other.mmData[2] * other.mmData[7]) * inverseDet;
		mmData[2] = (other.mmData[1] * other.mmData[5] - other.mmData[2] * other.mmData[4]) * inverseDet;
		mmData[3] = -(other.mmData[3] * other.mmData[8] - other.mmData[5] * other.mmData[6]) * inverseDet;
		mmData[4] = (other.mmData[0] * other.mmData[8] - t6) * inverseDet;
		mmData[5] = -(t2 - t4) * inverseDet;
		mmData[6] = (other.mmData[3] * other.mmData[7] - other.mmData[4] * other.mmData[6]) * inverseDet;
		mmData[7] = -(other.mmData[0] * other.mmData[7] - t5) * inverseDet;
		mmData[8] = (t1 - t3) * inverseDet;

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
	mmData[0] = other.mmData[0];
	mmData[1] = other.mmData[3];
	mmData[2] = other.mmData[6];
	mmData[3] = other.mmData[1];
	mmData[4] = other.mmData[4];
	mmData[5] = other.mmData[7];
	mmData[6] = other.mmData[2];
	mmData[7] = other.mmData[5];
	mmData[8] = other.mmData[8];
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