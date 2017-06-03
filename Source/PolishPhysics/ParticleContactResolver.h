#pragma once
#include <cstdint>
#include "Precision.h"
#include "ParticleContact.h"
namespace PolishPhysics
{
	/**Handles resolution of all contacts in the simulation. One instance can be shared for the entire
	simulation.*/
	class ParticleContactResolver
	{
	protected:

		/**Holds the maximum number of iterations. */
		std::uint32_t mIterations;

		/** */
		std::uint32_t mIterationsUsed;

	public:

		/**Creates a new contact resolver. 
		* @param iterations- The maximum number of iterations.*/
		ParticleContactResolver(std::uint32_t iterations);

		/**Sets the maximum number of iterations 
		* @param iterations - The new maximum number of iterations*/
		void SetMaxIterations(std::uint32_t iterations);

		/**Gets the maximum number of iterations.
		* @return - A precision that represents the maximum number of iterations.*/
		Precision GetMaxIterations();

		/**Resolves the set of particle contacts for the current frame.
		* @param contactArray - The list of collisions.
		* @param numContacts - The number of collisions.
		* @param deltaTime - The time since the last frame.*/
		void ResolveContacts(ParticleContact* contactArray, std::uint32_t numContacts, Precision deltaTime);
	};
}