#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Vector3::Vector3() :
	X(0), Y(0), Z(0)
{

}

Vector3::Vector3(Precision x, Precision y, Precision z) :
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

Precision Vector3::Magnitude() const
{
	return static_cast<Precision> (sqrt(X*X + Y*Y + Z*Z));
}

Precision Vector3::SquareMagnitude() const
{
	return X*X + Y*Y + Z*Z;
}

void Vector3::Normalize()
{
	Precision magnitude = Magnitude();

	if (magnitude > 0)
	{
		X /= magnitude;
		Y /= magnitude;
		Z /= magnitude;
	}
}

void Vector3::operator*=(Precision scalar)
{
	X *= scalar;
	Y *= scalar;
	Z *= scalar;
}

Vector3 Vector3::operator*(Precision scalar) const
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

void Vector3::AddScaledVector(const Vector3& other, Precision scale)
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

Precision Vector3::ScalarProduct(const Vector3& other) const
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

void Vector3::Clear()
{
	*this = ZeroVector();
}

bool Vector3::MakeOrthonormalBasis(Vector3& a, Vector3& b, Vector3& c)
{
	bool success = false;

	a.Normalize();
	c = a.VectorProduct(b);

	if (c.SquareMagnitude() != 0.0)
	{
		c.Normalize();
		b = c.VectorProduct(a);
		success = true;
	}

	return success;
}

Vector3 Vector3::ZeroVector()
{
	return Vector3(0.0f, 0.0f, 0.0f);
}