#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Matrix4::Matrix4()
{
	mData[1] = mData[2] = mData[3] = mData[4] = mData[6] =
		mData[7] = mData[8] = mData[9] = mData[11] = 0;
	mData[0] = mData[5] = mData[10] = 1;
}

Vector3 Matrix4::operator*(const Vector3& vector) const
{
	return Vector3(vector.X*mData[0] + vector.Y*mData[1] + vector.Z*mData[2] + mData[3],
		vector.X*mData[4] + vector.Y*mData[5] + vector.Z*mData[6] + mData[7],
		vector.X*mData[8] + vector.Y*mData[9] + vector.Z*mData[10] + mData[11]);
}

Vector3 Matrix4::Transform(const Vector3& vector) const
{
	return (*this) * vector;
}

Vector3 Matrix4::TransformInverse(const Vector3& vector) const
{
	Vector3 temp = vector;

	temp.X -= mData[3];
	temp.Y -= mData[7];
	temp.Z -= mData[11];

	return Vector3(temp.X * mData[0] + temp.Y *mData[4] * temp.Z *mData[8],
		temp.X * mData[1] + temp.Y *mData[5] * temp.Z *mData[9],
		temp.X * mData[2] + temp.Y *mData[6] * temp.Z *mData[10]);
}

Vector3 Matrix4::TransformDirection(const Vector3& vector) const
{
	return Vector3(vector.X*mData[0] + vector.Y*mData[1] + vector.Z*mData[2],
		vector.X * mData[4] + vector.Y * mData[5] + vector.Z * mData[6],
		vector.X * mData[8] + vector.Y * mData[9] + vector.Z * mData[10]);
}

Vector3 Matrix4::TransformInverseDirection(const Vector3& vector) const
{
	return Vector3(vector.X*mData[0] + vector.Y*mData[4] + vector.Z*mData[8],
		vector.X * mData[1] + vector.Y * mData[5] + vector.Z * mData[9],
		vector.X * mData[2] + vector.Y * mData[6] + vector.Z * mData[10]);
}

Matrix4 Matrix4::operator*(const Matrix4& other) const
{
	Matrix4 result;

	result.mData[0] = other.mData[0] * mData[0] + other.mData[4] * mData[1] + other.mData[8] * mData[2];
	result.mData[4] = other.mData[0] * mData[4] + other.mData[4] * mData[5] + other.mData[8] * mData[6];
	result.mData[8] = other.mData[0] * mData[8] + other.mData[4] * mData[9] + other.mData[8] * mData[10];

	result.mData[1] = other.mData[1] * mData[0] + other.mData[5] * mData[1] + other.mData[9] * mData[2];
	result.mData[5] = other.mData[1] * mData[4] + other.mData[5] * mData[5] + other.mData[9] * mData[6];
	result.mData[9] = other.mData[1] * mData[8] + other.mData[5] * mData[9] + other.mData[9] * mData[10];

	result.mData[2] = other.mData[2] * mData[0] + other.mData[6] * mData[1] + other.mData[10] * mData[2];
	result.mData[6] = other.mData[2] * mData[4] + other.mData[6] * mData[5] + other.mData[10] * mData[6];
	result.mData[10] = other.mData[2] * mData[8] + other.mData[6] * mData[9] + other.mData[10] * mData[10];

	result.mData[3] = other.mData[3] * mData[0] + other.mData[7] * mData[1] + other.mData[11] * mData[2];
	result.mData[7] = other.mData[3] * mData[4] + other.mData[7] * mData[5] + other.mData[11] * mData[6];
	result.mData[11] = other.mData[3] * mData[8] + other.mData[7] * mData[9] + other.mData[11] * mData[10];

	return result;
}

Matrix4 Matrix4::Inverse() const
{
	Matrix4 result;
	result.SetInverse(*this);

	return result;
}

void Matrix4::Invert()
{
	SetInverse(*this);
}

Precision Matrix4::GetDeterminant() const
{
	return mData[8] * mData[5] * mData[2] +
		mData[4] * mData[9] * mData[2] +
		mData[8] * mData[1] * mData[6] -
		mData[0] * mData[9] * mData[6] -
		mData[4] * mData[1] * mData[10] +
		mData[0] * mData[5] * mData[10];

}

void Matrix4::SetInverse(const Matrix4& other)
{
	Precision det = other.GetDeterminant();

	if (det != 0.0f)
	{
		det = 1.0f / det;

		mData[0] = (-other.mData[9] * other.mData[6] + other.mData[5] * other.mData[10])*det;
		mData[4] = (other.mData[8] * other.mData[6] - other.mData[4] * other.mData[10])*det;
		mData[8] = (-other.mData[8] * other.mData[5] + other.mData[4] * other.mData[9])*det;

		mData[1] = (other.mData[9] * other.mData[2] - other.mData[1] * other.mData[10])*det;
		mData[5] = (-other.mData[8] * other.mData[2] + other.mData[0] * other.mData[10])*det;
		mData[9] = (other.mData[8] * other.mData[1] - other.mData[0] * other.mData[9])*det;

		mData[2] = (-other.mData[5] * other.mData[2] + other.mData[1] * other.mData[6])*det;
		mData[6] = (+other.mData[4] * other.mData[2] - other.mData[0] * other.mData[6])*det;
		mData[10] = (-other.mData[4] * other.mData[1] + other.mData[0] * other.mData[5])*det;

		mData[3] = (other.mData[9] * other.mData[6] * other.mData[3]
			- other.mData[5] * other.mData[10] * other.mData[3]
			- other.mData[9] * other.mData[2] * other.mData[7]
			+ other.mData[1] * other.mData[10] * other.mData[7]
			+ other.mData[5] * other.mData[2] * other.mData[11]
			- other.mData[1] * other.mData[6] * other.mData[11])*det;
		mData[7] = (-other.mData[8] * other.mData[6] * other.mData[3]
			+ other.mData[4] * other.mData[10] * other.mData[3]
			+ other.mData[8] * other.mData[2] * other.mData[7]
			- other.mData[0] * other.mData[10] * other.mData[7]
			- other.mData[4] * other.mData[2] * other.mData[11]
			+ other.mData[0] * other.mData[6] * other.mData[11])*det;
		mData[11] = (other.mData[8] * other.mData[5] * other.mData[3]
			- other.mData[4] * other.mData[9] * other.mData[3]
			- other.mData[8] * other.mData[1] * other.mData[7]
			+ other.mData[0] * other.mData[9] * other.mData[7]
			+ other.mData[4] * other.mData[1] * other.mData[11]
			- other.mData[0] * other.mData[5] * other.mData[11])*det;
	}
}

Vector3 Matrix4::LocalToWorld(const Vector3& local, const Matrix4& transform) const
{
	return transform.Transform(local);
}

Vector3 Matrix4::WorldToLocal(const Vector3& world, const Matrix4& transform) const
{
	return transform.TransformInverse(world);
}

Vector3 Matrix4::LocalToWorldDirection(const Vector3& local, const Matrix4& transform) const
{
	return transform.TransformDirection(local);
}

Vector3 Matrix4::WorldToLocalDirection(const Vector3& world, const Matrix4& transform) const
{
	return transform.TransformInverseDirection(world);
}

void Matrix4::SetOrientationAndPosition(const Quaternion& quaternion, const Vector3& position)
{
	mData[0] = 1 - (2 * quaternion.j * quaternion.j + 2 * quaternion.k * quaternion.k);
	mData[1] = 2 * quaternion.i * quaternion.j + 2 * quaternion.k * quaternion.r;
	mData[2] = 2 * quaternion.i * quaternion.k - 2 * quaternion.j * quaternion.r;
	mData[3] = position.X;

	mData[4] = 2 * quaternion.i * quaternion.j - 2 * quaternion.k * quaternion.r;
	mData[5] = 1 - (2 * quaternion.i * quaternion.i + 2 * quaternion.k * quaternion.k);
	mData[6] = 2 * quaternion.j * quaternion.k + 2 * quaternion.i * quaternion.r;
	mData[7] = position.Y;

	mData[8] = 2 * quaternion.i * quaternion.k + 2 * quaternion.j * quaternion.r;
	mData[9] = 2 * quaternion.j * quaternion.k - 2 * quaternion.i * quaternion.r;
	mData[10] = 1 - (2 * quaternion.i * quaternion.i + 2 * quaternion.j * quaternion.j);
	mData[11] = position.Z;
}

Vector3 Matrix4::GetAxisVector(std::int32_t i) const
{
	return Vector3(mData[i], mData[i + 4], mData[i + 8]);
}
