#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Quaternion::Quaternion(Precision w /* = 0 */, Precision x /* = 0 */, Precision y /* = 0 */, Precision z /* = 0 */)
{
	r = w;
	i = x;
	j = y;
	k = z;
}

void Quaternion::Normalize()
{
	Precision magnitude = r*r + i*i + j*j + k*k;

	if (magnitude != 0.0f)
	{
		magnitude = 1.0f / precision_sqrt(magnitude);

		r *= magnitude;
		i *= magnitude;
		j *= magnitude;
		k *= magnitude;
	}
	else
	{
		r = 1.0f;
	}
}

void Quaternion::operator*=(const Quaternion& other)
{
	Quaternion lhs = *this;

	r = lhs.r * other.r - lhs.i * other.i - lhs.j * other.j - lhs.k * other.k;
	i = lhs.r * other.i + lhs.i * other.r + lhs.j * other.k - lhs.k * other.j;
	j = lhs.r * other.j + lhs.j * other.r + lhs.k * other.i - lhs.i * other.k;
	k = lhs.r * other.k + lhs.k * other.r + lhs.i * other.j - lhs.j * other.i;
}

void Quaternion::RotateByVector(const Vector3& vector)
{
	Quaternion rotationQuaternion(0, vector.X, vector.Y, vector.Z);
	*this *= rotationQuaternion;
}

void Quaternion::AddScaledVector(const Vector3& vector, Precision scale)
{
	Quaternion rotationQuaternion(0, vector.X * scale, vector.Y * scale, vector.Z * scale);
	
	rotationQuaternion *= *this;

	r += rotationQuaternion.r * static_cast<Precision> (0.5f);
	i += rotationQuaternion.i * static_cast<Precision> (0.5f);
	j += rotationQuaternion.j * static_cast<Precision> (0.5f);
	k += rotationQuaternion.k * static_cast<Precision> (0.5f);

}