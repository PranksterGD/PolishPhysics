#include "pch.h"

using namespace std;
using namespace PolishPhysics;

void ParticleContact::Resolve(Precision deltaTime)
{
	ResolveVelocity(deltaTime);
	ResolveInterpenetration(deltaTime);
}

Precision ParticleContact::GetSeperatingVelocity() const
{
	Vector3 relativeVelocity = mParticles[0]->GetVelocity();

	if (mParticles[1] != nullptr)
	{
		relativeVelocity -= mParticles[1]->GetVelocity();
	}

	return  relativeVelocity.ScalarProduct(mContactNormal);
}

void ParticleContact::ResolveVelocity(Precision deltaTime)
{
	Precision seperatingVelocity = GetSeperatingVelocity();

	//Early out if contact is either separating or stationary
	if (seperatingVelocity < 0.0f)
	{
		//Calculate new separating velocity based on restitution.
		Precision newSeperatingVelocity = -seperatingVelocity * mRestitution;

		//Check the velocity buildup due to acceleration only.

		Vector3 accCausedVelocity = mParticles[0]->GetAcceleration();

		if (mParticles[1] != nullptr)
		{
			accCausedVelocity -= mParticles[1]->GetAcceleration;
		}

		Precision accCausedSepVelocity = accCausedVelocity.ScalarProduct(mContactNormal) * deltaTime;

		//If we've got a closing velocity due to acceleration buildup, remove it from the new
		//separating velocity - This is done to deal with resting contacts.

		if (accCausedSepVelocity < 0.0f)
		{
			newSeperatingVelocity += mRestitution * accCausedSepVelocity;

			if (newSeperatingVelocity < 0.0f)
			{
				newSeperatingVelocity = 0.0f;
			}
		}

		Precision velocityChange = newSeperatingVelocity - seperatingVelocity;

		//Apply change in velocity in proportion to inverse mass.
		Precision totalInverseMass = mParticles[0]->GetInverseMass();

		if (mParticles[1] != nullptr)
		{
			totalInverseMass += mParticles[1]->GetInverseMass();
		}

		if (totalInverseMass > 0.0f)
		{
			Precision impulseperInverseMass = velocityChange / totalInverseMass;

			Vector3 impulse = mContactNormal * impulseperInverseMass;

			mParticles[0]->SetVelocity(mParticles[0]->GetVelocity() + impulse * mParticles[0]->GetInverseMass());

			if (mParticles[1] != nullptr)
			{
				mParticles[1]->SetVelocity(mParticles[1]->GetVelocity() + impulse* -mParticles[1]->GetInverseMass());
			}
		}
	}
}

void ParticleContact::ResolveInterpenetration(Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	Vector3 particleMovement[2];

	//Early out if we're not penetrating.
	if (mPenetration > 0.0f)
	{
		Precision totalInverseMass = mParticles[0]->GetInverseMass();

		if (mParticles[1] != nullptr)
		{
			totalInverseMass += mParticles[1]->GetInverseMass();
		}

		//Early out if both objects are of infinite mass.
		if (totalInverseMass > 0.0f)
		{
			//Change position proportional to inverse mass.
			Vector3 movementPerInverseMass = mContactNormal * (mPenetration / totalInverseMass);

			particleMovement[0] = movementPerInverseMass * mParticles[0]->GetInverseMass();

			if (mParticles[1] != nullptr)
			{
				particleMovement[1] = movementPerInverseMass * -mParticles[1]->GetInverseMass();
			}
			else
			{
				particleMovement[1].Clear();
			}

			mParticles[0]->SetPosition(mParticles[0]->GetPosition() + particleMovement[0]);

			if (mParticles[1] != nullptr)
			{
				mParticles[1]->SetPosition(mParticles[1]->GetPosition() + particleMovement[1]);
			}
		}
	}
}