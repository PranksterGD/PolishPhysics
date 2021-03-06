#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that applies a spring force. To denote a 2 way connection between 2 particles,
	Create one spring force generator for each particle with the other particle being passed in as the
	mOtherParticle.*/
	class ParticleSpringForceGenerator : public ParticleForceGenerator
	{
	private:

		/**The particle at the other end of the spring. */
		const Particle* mOtherParticle;

		/**Holds the spring constant for the spring. */
		Precision mSpringConstant;

		/**Holds the rest length of the spring. */
		Precision mRestLength;

	public:

		/**Creates a new spring with the given parameters. */
		ParticleSpringForceGenerator(const Particle& otherParticle, Precision springConstant, Precision restLength);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}