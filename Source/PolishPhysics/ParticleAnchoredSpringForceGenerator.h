#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	class ParticleAnchoredSpringForceGenerator : public ParticleForceGenerator
	{
	private:

		/**The position of the anchor. */
		Vector3 mAnchor;

		/**Holds the spring constant for the spring. */
		Precision mSpringConstant;

		/**Holds the rest length of the spring. */
		Precision mRestLength;

	public:

		ParticleAnchoredSpringForceGenerator(Vector3 anchor, Precision springConstant, Precision restLength);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);

		void SetAnchorPosition(const Vector3& anchor);

		Vector3 GetAnchorPosition() const;
	};
}