#pragma once
#include <vector>
#include <cstdint>
#include "RigidBodyForceRegistry.h"
namespace PolishPhysics
{
	/**Holds all the particles in the world and provides an interface to update them. */
	class RigidBodyWorld
	{
	public:

		typedef std::vector<class RigidBody*> RigidBodies;

	protected:

		RigidBodies mBodies;

		RigidBodyForceRegistry mRegistry;

	public:

		/**Creates a new rigid body simulation.*/
		RigidBodyWorld();

		void AddBody(RigidBody& body);

		void RegisterForce(RigidBody& body, RigidBodyForceGenerator& generator);

		/**Prepares the simulation to process the next frame. */
		void StartFrame();

		/**Updates all elements of the physics simulation.
		* @param deltaTime - Time since the last frame.*/
		void Update(Precision deltaTime);

		/**Frees memory and destroys the particle world. */
		~RigidBodyWorld();

	private:

		/**Integrates all particles in the world forward in time.
		* @param deltaTime - Time since the last frame.*/
		void Integrate(Precision deltaTime);
	};
}