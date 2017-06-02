#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleAnchoredSpringForceGenerator::ParticleAnchoredSpringForceGenerator(Vector3 anchor, Precision springConstant, Precision restLength) :
	mAnchor(anchor), mSpringConstant(springConstant), mRestLength(restLength)
{

}

void ParticleAnchoredSpringForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	Vector3 force = particle.GetPosition();
	force -= mAnchor;

	Precision magnitude = force.Magnitude();
	magnitude = (mRestLength - magnitude) * mSpringConstant;

	force.Normalize();
	force *= -magnitude;

	particle.AddForce(force);
}

void ParticleAnchoredSpringForceGenerator::SetAnchorPosition(const Vector3& anchor)
{
	mAnchor = anchor;
}

Vector3 ParticleAnchoredSpringForceGenerator::GetAnchorPosition() const
{
	return mAnchor;
}