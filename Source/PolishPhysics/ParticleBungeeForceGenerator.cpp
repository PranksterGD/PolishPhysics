#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleBungeeForceGenerator::ParticleBungeeForceGenerator(const Particle& otherParticle, Precision springConstant, Precision restLength) :
	mOtherParticle(&otherParticle), mSpringConstant(springConstant), mRestLength(restLength)
{

}

void ParticleBungeeForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	Vector3 force = particle.GetPosition();
	force -= mOtherParticle->GetPosition();

	Precision magnitude = force.Magnitude();

	//Ensure bungee is not compressed.
	if (magnitude > mRestLength)
	{
		magnitude = mSpringConstant * (mRestLength - magnitude);

		force.Normalize();
		force *= -magnitude;

		particle.AddForce(force);
	}
}