#include "pch.h"

using namespace std;
using namespace PolishPhysics;


void RigidBodyForceRegistry::Add(RigidBody& body, RigidBodyForceGenerator& generator)
{
	RigidBodyForceRegistration registration;
	registration.mGenerator = &generator;
	registration.mBody = &body;

	mRegistrations.push_back(registration);
}

void RigidBodyForceRegistry::Remove(RigidBody& body, RigidBodyForceGenerator& generator)
{
	for (auto it = mRegistrations.begin(); it != mRegistrations.end(); ++it)
	{
		if (it->mGenerator == &generator && it->mBody == &body)
		{
			mRegistrations.erase(it, it);
		}
	}
}

void RigidBodyForceRegistry::Clear()
{
	mRegistrations.clear();
}

void RigidBodyForceRegistry::UpdateForces(Precision deltaTime)
{
	for (auto it = mRegistrations.begin(); it != mRegistrations.end(); ++it)
	{
		it->mGenerator->UpdateForce(*it->mBody, deltaTime);
	}
}