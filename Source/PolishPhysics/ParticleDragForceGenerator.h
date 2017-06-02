#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that applies a drag force. One instance can be used for multiple particles. */
	class ParticleDragForceGenerator : public ParticleForceGenerator
	{
	private:

		/**Holds the velocity drag coefficient. */
		Precision mK1;
		
		/**Holds the velocity squared drag coefficient. */
		Precision mK2;

	public:

		/**Creates a new generator with the given parameters. */
		ParticleDragForceGenerator(Precision k1, Precision k2);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}