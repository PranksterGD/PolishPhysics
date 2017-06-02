#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	/**A force generator that resembles a stiff spring force, where one end is attached
	to a fixed point in space.*/
	class ParticleStiffSpringForceGenerator : public ParticleForceGenerator
	{
	private:

		/**The position of the anchor. */
		Vector3 mAnchor;

		/**Holds the spring constant for the spring. */
		Precision mSpringConstant;

		/**Holds the damping on the oscillation of the spring. */
		Precision mDamping;

	public:

		/**Creates a new spring with the given parameters. */
		ParticleStiffSpringForceGenerator(const Vector3& anchor, Precision springConstant, Precision damping);

		/**Applies the force to the given particle.
		* @param particle - The particle on which the force is to be applied to.
		* @param deltaTime- Time since the last frame.*/
		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}