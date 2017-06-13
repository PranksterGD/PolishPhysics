#pragma once
#include "RigidBodyForceGenerator.h"
#include "Particle.h"
#include "RigidBody.h"
#include <vector>
namespace PolishPhysics
{
	/**Holds all the force generators and the particles that they apply to. */
	class RigidBodyForceRegistry
	{
	protected:

		struct RigidBodyForceRegistration
		{
			RigidBody* mBody;
			RigidBodyForceGenerator* mGenerator;
		};

		/**Holds the list of registrations*/

		typedef std::vector<RigidBodyForceRegistration> Registry;
		Registry mRegistrations;

	public:

		/**Registers the given force generator to the given particle. */
		void Add(RigidBody& body, RigidBodyForceGenerator& generator);

		/**Removes the given registration. */
		void Remove(RigidBody& body, RigidBodyForceGenerator& generator);

		/**Removes all registrations. */
		void Clear();

		/**Calls update force on all generators. */
		void UpdateForces(Precision deltaTime);

	};
}
