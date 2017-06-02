#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleGravityForceGenerator::ParticleGravityForceGenerator(const Vector3& gravity) :
	mGravity(gravity)
{

}

void ParticleGravityForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	if (!particle.HasInfiniteMass())
	{
		particle.AddForce(mGravity* particle.GetInverseMass());
	}
}