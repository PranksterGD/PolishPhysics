#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that applies a spring force where one end is attached to a fixed point in space.
	Only applies the force when extended.*/
	class ParticleAnchoredBungeeForceGenerator : public ParticleForceGenerator
	{
	private:

		/**The position of the anchor. */
		Vector3 mAnchor;

		/**Holds the spring constant for the spring. */
		Precision mSpringConstant;

		/**Holds the rest length of the spring. */
		Precision mRestLength;

	public:
		/**Creates a new spring with the given parameters. */
		ParticleAnchoredBungeeForceGenerator(const Vector3& anchor, Precision springConstant, Precision restLength);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);

		void SetAnchorPosition(const Vector3& anchor);

		Vector3 GetAnchorPosition() const;
	};
}