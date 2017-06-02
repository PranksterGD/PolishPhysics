#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that applies a buoyancy force for a plane of liquid parallel to the XZ plane. */
	class ParticleBuoyancyForceGenerator : public ParticleForceGenerator
	{
	private:

		/**Holds the depth at which the particle has maximum buoyancy force.. */
		Precision mMaxDepth;

		/**Holds the volume of the particle. */
		Precision mVolume;

		/**Holds the height of the liquid plane along the Y axis. */
		Precision mLiquidHeight;

		/**Holds the density of the liquid. Defaulted to water = 1000 */
		Precision mLiquidDensity;

	public:

		/**Creates a new generator with the given parameters. */
		ParticleBuoyancyForceGenerator(Precision maxDepth, Precision volume,  Precision liquidHeight, Precision liquidDensity = 1000.0f);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}