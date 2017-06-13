#pragma once
#include "RigidBodyForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that applies a spring force. To denote a 2 way connection between 2 particles,
	Create one spring force generator for each particle with the other particle being passed in as the
	mOtherParticle.*/
	class RigidBodySpringForceGenerator : public RigidBodyForceGenerator
	{
	private:

		/**The particle at the other end of the spring. */
		const RigidBody* mOtherBody;

		/**Holds the spring constant for the spring. */
		Precision mSpringConstant;

		/**Holds the rest length of the spring. */
		Precision mRestLength;

		/**Holds the point of connection of the spring in local coordinates */
		Vector3 mConnectionPoint;

		/**Holds the point of connection of the spring to the other object in local coordinates */
		Vector3 mOtherConnectionPoint;

	public:

		/**Creates a new spring with the given parameters. */
		RigidBodySpringForceGenerator(const RigidBody& otherBody, Precision springConstant, Precision restLength,
			const Vector3& connectionPoint, const Vector3& otherConnectionPoint);

		virtual void UpdateForce(class RigidBody& otherBody, Precision deltaTime);
	};
}