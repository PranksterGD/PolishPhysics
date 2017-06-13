#pragma once
#include "RigidBodyForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that applies a gravitational force in the -Y direction.
	One instance can be used for multiple particles that are expected to behave the same way to gravity.*/
	class RigidBodyGravityForceGenerator : public RigidBodyForceGenerator
	{
	private:

		/**Holds the acceleration due to gravity. */
		Vector3 mGravity;

	public:

		/**Creates a new generator with the given parameters. */
		RigidBodyGravityForceGenerator(const Vector3& gravity);

		virtual void UpdateForce(class RigidBody& body, Precision deltaTime);
	};
}
