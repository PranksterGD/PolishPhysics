#pragma once
#include "ParticleForceGenerator.h"
#include "Particle.h"
#include <vector>
namespace PolishPhysics
{
	/**Holds all the force generators and the particles that they apply to. */
	class ParticleForceRegistry
	{
	protected:

		struct ParticleForceRegistration
		{
			Particle* mParticle;
			ParticleForceGenerator* mGenerator;
		};

		/**Holds the list of registrations*/

		typedef std::vector<ParticleForceRegistration> Registry;
		Registry mRegistrations;

	public:

		/**Registers the given force generator to the given particle. */
		void Add(Particle& particle, ParticleForceGenerator& generator);

		/**Removes the given registration. */
		void Remove(Particle& particle, ParticleForceGenerator& generator);

		/**Removes all registrations. */
		void Clear();

		/**Calls update force on all generators. */
		void UpdateForces(Precision deltaTime);

	};
}
