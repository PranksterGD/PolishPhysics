#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleSpringForceGenerator::ParticleSpringForceGenerator(Particle& otherParticle, Precision springConstant, Precision restLength) :
	mOtherParticle(&otherParticle), mSpringConstant(springConstant), mRestLength(restLength)
{

}

void ParticleSpringForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	Vector3 force = particle.GetPosition();
	force -= mOtherParticle->GetPosition();

	Precision magnitude = force.Magnitude();
	magnitude = precision_abs(magnitude - mRestLength);
	magnitude *= mSpringConstant;

	force.Normalize();
	force *= -magnitude;

	particle.AddForce(force);
}