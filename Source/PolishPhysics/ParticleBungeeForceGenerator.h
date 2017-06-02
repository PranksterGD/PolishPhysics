#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that applies a spring force only when extended. */
	class ParticleBungeeForceGenerator : public ParticleForceGenerator
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
		ParticleBungeeForceGenerator(const Particle& otherParticle, Precision springConstant, Precision restLength);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}