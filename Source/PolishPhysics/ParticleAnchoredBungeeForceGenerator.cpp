#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleAnchoredBungeeForceGenerator::ParticleAnchoredBungeeForceGenerator(const Vector3& anchor, Precision springConstant, Precision restLength) :
	mAnchor(anchor), mSpringConstant(springConstant), mRestLength(restLength)
{

}

void ParticleAnchoredBungeeForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	Vector3 force = particle.GetPosition();
	force -= mAnchor;

	Precision magnitude = force.Magnitude();

	//Ensure bungee is not compressed.
	if (magnitude > mRestLength)
	{
		magnitude = (mRestLength - magnitude) * mSpringConstant;

		force.Normalize();
		force *= -magnitude;

		particle.AddForce(force);
	}
}

void ParticleAnchoredBungeeForceGenerator::SetAnchorPosition(const Vector3& anchor)
{
	mAnchor = anchor;
}

Vector3 ParticleAnchoredBungeeForceGenerator::GetAnchorPosition() const
{
	return mAnchor;
}