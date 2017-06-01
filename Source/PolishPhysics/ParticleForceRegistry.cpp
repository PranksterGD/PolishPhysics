#include "pch.h"

using namespace std;
using namespace PolishPhysics;

void ParticleForceRegistry::Add(Particle& particle, ParticleForceGenerator& generator)
{
	ParticleForceRegistration registration;
	registration.mGenerator = &generator;
	registration.mParticle = &particle;

	mRegistrations.push_back(registration);
}

void ParticleForceRegistry::Remove(Particle& particle, ParticleForceGenerator& generator)
{
	for (auto it = mRegistrations.begin(); it != mRegistrations.end(); ++it)
	{
		if (it->mGenerator == &generator && it->mParticle == &particle)
		{
			mRegistrations.erase(it, it);
		}
	}
}

void ParticleForceRegistry::Clear()
{
	mRegistrations.clear();
}

void ParticleForceRegistry::UpdateForces(Precision deltaTime)
{
	for (auto it = mRegistrations.begin(); it != mRegistrations.end(); ++it)
	{
		it->mGenerator->UpdateForce(*it->mParticle, deltaTime);
	}
}