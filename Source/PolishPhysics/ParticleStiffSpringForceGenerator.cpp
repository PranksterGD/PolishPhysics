#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleStiffSpringForceGenerator::ParticleStiffSpringForceGenerator(const Vector3& anchor, Precision springConstant, Precision damping) :
	mAnchor(anchor), mSpringConstant(springConstant), mDamping(damping)
{

}

void ParticleStiffSpringForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	//Check that we do not have infinite mass.
	if (!particle.HasInfiniteMass())
	{
		Vector3 position = particle.GetPosition();
		position -= mAnchor;

		//Calculate gamma and c
		Precision gamma = 0.5f* precision_sqrt(4 * mSpringConstant - mDamping * mDamping);

		if (gamma != 0.0f)
		{
			Vector3 c = position * (mDamping / (2.0f* gamma)) + particle.GetVelocity() * (1.0f / gamma);

			//Calculate target position.

			Vector3 targetPos = position * precision_cos(gamma * deltaTime) + c * precision_sin(gamma * deltaTime);
			targetPos *= precision_exp(-0.5f* mDamping* deltaTime);


			//Calculate acceleration and force based on target position.

			Vector3 accelration = (targetPos - position) * (1.0f / deltaTime * deltaTime) -
				particle.GetVelocity() * deltaTime;

			particle.AddForce(accelration * particle.GetMass());
		}
	}
}