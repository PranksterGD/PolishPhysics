#pragma once
#include "Precision.h"
namespace PolishPhysics
{
	class ParticleForceGenerator
	{
	public:

		/**Overload this implementation to caluclate and update the force applied to the given particle.
		* @param particle- The particle who's force is to be updated.
		* @param deltaTime- The time since the last frame.*/
		virtual void UpdateForce(class Particle& particle, Precision deltaTime) = 0;
	};
}