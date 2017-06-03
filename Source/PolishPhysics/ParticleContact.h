#pragma once
#include "Vector3.h"

namespace PolishPhysics
{
	/**Holds contact information about a collision between 2 particles. */
	class ParticleContact
	{

		friend class ParticleContactResolver;
	public:

		/**Holds the particles involved in the collision. The second particle can be nullptr if
		colliding with the environment.*/
		class Particle* mParticles[2];

		/**Holds the normal restitution coefficient. */
		Precision mRestitution;

		/**Holds the direction  of the contact normal. */
		Vector3 mContactNormal;

		/**Holds the depth of penetration at the contact. */
		Precision mPenetration;

	protected:

		/**Resolves the contact for both velocity and inter penetration.
		* @param deltaTime - The time since the last frame.*/
		void Resolve(Precision deltaTime);

		/**Calculates the separating velocity at the point of contact.
		* @return - A precision that represents the magnitude of the separating velocity.*/
		Precision GetSeperatingVelocity() const;

	private:

		/**Handles the impulse calculations for the collision.
		* @param deltaTime - The time since the last frame.*/
		void ResolveVelocity(Precision deltaTime);

		/**Handles the inter penetration resolution for the collision.
		* @param deltaTime- The time since the last frame.*/
		void ResolveInterpenetration(Precision deltaTime);
	};
}