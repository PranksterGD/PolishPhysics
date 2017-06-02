#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	class ParticleSpringForceGenerator : public ParticleForceGenerator
	{
	private:

		/**The particle at the other end of the spring. */
		Particle* mOtherParticle;

		/**Holds the spring constant for the spring. */
		Precision mSpringConstant;

		/**Holds the rest length of the spring. */
		Precision mRestLength;

	public:

		ParticleSpringForceGenerator(Particle& otherParticle, Precision springConstant, Precision restLength);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}