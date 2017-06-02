#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	class ParticleDragForceGenerator : public ParticleForceGenerator
	{
	private:

		/**Holds the velocity drag coefficient. */
		Precision mK1;
		
		/**Holds the velocity squared drag coefficient. */
		Precision mK2;

	public:

		ParticleDragForceGenerator(Precision k1, Precision k2);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}