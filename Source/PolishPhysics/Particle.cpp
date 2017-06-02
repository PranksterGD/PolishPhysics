#include "pch.h"

using namespace PolishPhysics;

Particle::Particle() :
	mPosition(Vector3::ZeroVector()), mVelocity(Vector3::ZeroVector()), mAcceleration(Vector3::ZeroVector()),
	mDamping(0.999f), mInverseMass(0.0f), mGravity(Vector3(0,-10,0))
{

}

Particle::Particle(const Particle& other) :
	mPosition(other.mPosition), mVelocity(other.mVelocity), mAcceleration(other.mAcceleration),
	mDamping(other.mDamping), mInverseMass(other.mInverseMass)
{

}

Particle::Particle(Particle&& other) :
	mPosition(other.mPosition), mVelocity(other.mVelocity), mAcceleration(other.mAcceleration),
	mDamping(other.mDamping), mInverseMass(other.mInverseMass)
{
	other = Particle();
}

Particle& Particle::operator=(const Particle& other)
{
	if (this != &other)
	{
		mPosition = other.mPosition;
		mVelocity = other.mVelocity;
		mAcceleration = other.mAcceleration;
		mDamping = other.mDamping;
		mInverseMass = other.mInverseMass;
	}

	return *this;
}

Particle& Particle::operator=(Particle&& other)
{
	if (this != &other)
	{
		mPosition = other.mPosition;
		mVelocity = other.mVelocity;
		mAcceleration = other.mAcceleration;
		mDamping = other.mDamping;
		mInverseMass = other.mInverseMass;

		other = Particle();
	}

	return *this;
}

bool Particle::operator==(const Particle& other) const
{
	return mPosition == other.mPosition &&
		mVelocity == other.mVelocity &&
		mAcceleration == other.mAcceleration &&
		mDamping == other.mDamping &&
		mInverseMass == other.mInverseMass;
}

bool Particle::operator!=(const Particle& other) const
{
	return !(*this == other);
}

void Particle::SetPosition(const Vector3& position)
{
	mPosition = position;
}

Vector3 Particle::GetPosition() const
{
	return mPosition;
}

void Particle::SetVelocity(const Vector3& velocity)
{
	mVelocity = velocity;
}

Vector3 Particle::GetVelocity() const
{
	return mVelocity;
}

void Particle::SetAcceleration(const Vector3& acceleration)
{
	mAcceleration = acceleration;
}

Vector3 Particle::GetAcceleration() const
{
	return mAcceleration;
}

void Particle::SetDamping(Precision damping)
{
	mDamping = damping;
}

Precision Particle::GetDamping() const
{
	return mDamping;
}

void Particle::SetMass(Precision mass)
{
	if (mass == 0.0)
	{
		mInverseMass = 0.0f;
	}
	else
	{
		mInverseMass = 1 / mass;
	}
}

Precision Particle::GetMass() const
{
	return 1 / mInverseMass;
}

void Particle::SetInverseMass(Precision inversemass)
{
	mInverseMass = inversemass;
}

Precision Particle::GetInverseMass() const
{
	return mInverseMass;
}

void Particle::SetGravity(const Vector3 gravity)
{
	mGravity = gravity;
}

Vector3 Particle::GetGravity() const
{
	return mGravity;
}

void Particle::Integrate(Precision deltaTime)
{
	//Don't update if infinite mass.
	if (mInverseMass > 0.0f)
	{
		assert(deltaTime > 0.0f);

		//Update linear position
		mPosition.AddScaledVector(mVelocity, deltaTime);

		//Calculate acceleration from force- TODO
		Vector3 newAcceleration = mAcceleration;
		newAcceleration.AddScaledVector(mAccumulatedForce, mInverseMass);

		newAcceleration += mGravity;

		mVelocity.AddScaledVector(newAcceleration, deltaTime);

		mVelocity *= precision_pow(mDamping, deltaTime);

		ClearAccumulator();
	}
}

void Particle::AddForce(const Vector3& force)
{
	mAccumulatedForce += force;
}

bool Particle::HasInfiniteMass() const
{
	return mInverseMass == 0;
}

void Particle::ClearAccumulator()
{
	mAccumulatedForce.Clear();
}