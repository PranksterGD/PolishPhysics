#pragma once
#include <vector>
#include <cstdint>
#include "ParticleForceRegistry.h"
#include "ParticleContactResolver.h"
namespace PolishPhysics
{
	/**Holds all the particles in the world and provides an interface to update them. */
	class ParticleWorld
	{
	public:

		typedef std::vector<class Particle*> Particles;

		typedef std::vector<class ParticleContactGenerator*> ContactGenerators;

	protected:

		Particles mParticles;

		ParticleForceRegistry mRegistry;

		ParticleContactResolver mResolver;

		ContactGenerators mContactGenerators;

		ParticleContact* mContacts;

		std::uint32_t mMaxContacts;

		bool mShouldCalculateIterations;

	public:

		/**Creates a new particle simulation.
		* @param maxContacts - The max number of contacts that can be handled per frame.
		* @param iterations - The number of iterations to be used for collision resolution.
		If no value is specified, twice the number of contact will be used.*/
		ParticleWorld(std::uint32_t maxContacts, std::uint32_t iterations = 0);

		/**Prepares the simulation to process the next frame. */
		void StartFrame();

		/**Calls each of the registered contact generators to report their contacts.
		* @return - An unsigned integer that represents the total number of contacts generated.*/
		std::uint32_t GenerateContacts();

		/**Integrates all particles in the world forward in time.
		* @param deltaTime - Time since the last frame.*/
		void Integrate(Precision deltaTime);

		/**Updates all elements of the physics simulation.
		* @param deltaTime - Time since the last frame.*/
		void Update(Precision deltaTime);

		/**Frees memory and destroys the particle world. */
		~ParticleWorld();
	};
}