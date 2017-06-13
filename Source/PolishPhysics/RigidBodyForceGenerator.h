#pragma once
#include "Precision.h"
#include "RigidBody.h"
namespace PolishPhysics
{
	class RigidBodyForceGenerator
	{
	public:

		/**Overload this implementation to calculate and update the force applied to the given particle.
		* @param particle- The particle who's force is to be updated.
		* @param deltaTime- The time since the last frame.*/
		virtual void UpdateForce(class RigidBody& body, Precision deltaTime) = 0;
	};
}
