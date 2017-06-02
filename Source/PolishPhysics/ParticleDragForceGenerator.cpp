#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleDragForceGenerator::ParticleDragForceGenerator(Precision k1, Precision k2) :
	mK1(k1), mK2(k2)
{

}

void ParticleDragForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	Vector3 force = particle.GetVelocity();

	Precision dragCoeff = force.Magnitude();
	dragCoeff = mK1 * dragCoeff + mK2 * dragCoeff*dragCoeff;

	force.Normalize();
	force *= dragCoeff;

	particle.AddForce(force);
}