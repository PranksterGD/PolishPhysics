#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Vector3::Vector3() :
	X(0), Y(0), Z(0)
{

}

Vector3::Vector3(precision x, precision y, precision z) :
	X(x), Y(y), Z(z)
{

}

Vector3::Vector3(const Vector3& other) :
	X(other.X), Y(other.Y), Z(other.Z)
{

}

Vector3::Vector3(Vector3&& other) :
	X(other.X), Y(other.Y), Z(other.Z)
{
	other.X = 0;
	other.Y = 0;
	other.Z = 0;
}

Vector3& Vector3::operator=(const Vector3& other)
{
	if (this != &other)
	{
		X = other.X;
		Y = other.Y;
		Z = other.Z;
	}

	return *this;
}

Vector3& Vector3::operator=(Vector3&& other)
{
	if (this != &other)
	{
		X = other.X;
		Y = other.Y;
		Z = other.Z;

		other.X = 0;
		other.Y = 0;
		other.Z = 0;
	}

	return *this;
}

bool Vector3::operator==(const Vector3& other) const
{
	return(X == other.X && Y == other.Y && Z == other.Z);
}

bool Vector3::operator!=(const Vector3& other) const
{
	return !(*this == other);
}

void Vector3::Invert()
{
	X = -X;
	Y = -Y;
	Z = -Z;
}

precision Vector3::Magnitude() const
{
	return static_cast<precision> (sqrt(X*X + Y*Y + Z*Z));
}

precision Vector3::SquareMagnitude() const
{
	return X*X + Y*Y + Z*Z;
}

void Vector3::Normalize()
{
	precision magnitude = Magnitude();

	if (magnitude > 0)
	{
		*this *= (1 / magnitude);
	}
}

void Vector3::operator*=(precision scalar)
{
	X *= scalar;
	Y *= scalar;
	Z *= scalar;
}

Vector3 Vector3::operator*(precision scalar) const
{
	return Vector3(X*scalar, Y*scalar, Z*scalar);
}

void Vector3::operator+=(const Vector3& other)
{
	X += other.X;
	Y += other.Y;
	Z += other.Z;
}

Vector3 Vector3::operator+(const Vector3& other) const
{
	return Vector3(X + other.X, Y + other.Y, Z + other.Z);
}

void Vector3::operator-=(const Vector3& other)
{
	X -= other.X;
	Y -= other.Y;
	Z -= other.Z;
}

Vector3 Vector3::operator-(const Vector3& other) const
{
	return Vector3(X - other.X, Y - other.Y, Z - other.Z);
}

void Vector3::AddScaledVector(const Vector3& other, precision scale)
{
	X += other.X * scale;
	Y += other.Y * scale;
	Z += other.Z * scale;
}

void Vector3::ComponentProductAssignment(const Vector3& other)
{
	X *= other.X;
	Y *= other.Y;
	Z *= other.Z;
}

Vector3 Vector3::ComponentProduct(const Vector3& other) const
{
	return Vector3(X * other.X, Y * other.Y, Z * other.Z);
}

precision Vector3::ScalarProduct(const Vector3& other) const
{
	return X * other.X + Y * other.Y + Z * other.Z;
}

Vector3 Vector3::VectorProduct(const Vector3& other) const
{
	return Vector3(Y * other.Z - Z * other.Y, 
				   Z * other.X - X * other.Z,
				   X * other.Y - Y * other.X);
}

void Vector3::VectorProductAssignment(const Vector3& other)
{
	*this = VectorProduct(other);
}