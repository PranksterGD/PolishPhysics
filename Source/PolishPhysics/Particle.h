#pragma once
#include "Vector3.h"

namespace PolishPhysics
{
	/**Simplest object in the physics simulation- A point mass that has no rotation. */
	class Particle
	{
		friend class ParticleWorld;

	public:

		/**Default constructor for the particle. */
		Particle();

		/**Copy constructor for the Particle.
		* @param other- The other Particle to be copied from. */
		Particle(const Particle& other);

		/**Move constructor for the Particle.
		* @param other - The other Particle to be moved to. */
		Particle(Particle&& other);

		/**Assignment operator for the Particle.
		* @param other - The other Particle to be copied from. */
		Particle& operator=(const Particle& other);

		/**Move assignment operator for the Particle.
		* @param other - The other Particle to be moved to. */
		Particle& operator=(Particle&& other);

		/**Equality operator for the Particle.
		* @param other - The other Particle to be compared against.
		* @return A boolean - True if the Particles are equal, false otherwise.*/
		bool operator==(const Particle& other) const;

		/**Inequality operator for the Particle.
		* @param other - The other Particles to be compared against.
		* @return A boolean - True if the Particles are not equal, false otherwise.*/
		bool operator!=(const Particle& other) const;

		/**Function used to set the position of the particle.
		* @param position- The new position of the particle.*/
		void SetPosition(const Vector3& position);

		/**Function that returns the position of the particle.
		* @return - A vector3 that is the position of the particle.*/
		Vector3 GetPosition() const;

		/**Function used to set the position of the particle.
		* @param position- The new position of the particle.*/
		void SetVelocity(const Vector3& velocity) ;

		/**Function that returns the velocity of the particle.
		* @return - A vector3 that is the velocity of the particle.*/
		Vector3 GetVelocity() const;

		/**Function used to set the acceleration of the particle.
		* @param velocity- The new acceleration of the particle.*/
		void SetAcceleration(const Vector3& acceleration);

		/**Function that returns the acceleration of the particle.
		* @return - A vector3 that is the acceleration of the particle.*/
		Vector3 GetAcceleration() const;

		/**Function used to set the damping of the particle.
		* @param damping- The new damping of the particle.*/
		void SetDamping(Precision damping);

		/**Function that returns the damping of the particle.
		* @return - A Precision that is the damping of the particle.*/
		Precision GetDamping() const;

		/**Function used to set the mass of the particle.
		* @param mass- The new mass of the particle.*/
		void SetMass(Precision mass);

		/**Function that returns the mass of the particle.
		* @return - A Precision that is the mass of the particle.*/
		Precision GetMass() const;

		/**Function used to set the inversemass of the particle.
		* @param inversemass- The new inversemass of the particle.*/
		void SetInverseMass(Precision inversemass);

		/**Function that returns the inversemass of the particle.
		* @return - A Precision that is the inversemass of the particle.*/
		Precision GetInverseMass() const;

		/**Function used to set the gravity of the particle.
		* @param velocity- The new gravity of the particle.*/
		void SetGravity(const Vector3 gravity);

		/**Function that returns the gravity of the particle.
		* @return - A vector3 that is the gravity of the particle.*/
		Vector3 GetGravity() const;

		/**Updates the particle forward in time by the given amount. */
		void Integrate(Precision deltaTime);

		/**Function that adds a force to the accumulator.
		* @param force - The force to be added.*/
		void AddForce(const Vector3& force);

		/**Function that checks if the particle has infinite mass.
		* @return - A boolean -True if the particle has infinite mass, false otherwise.*/
		bool HasInfiniteMass() const;

	protected:

		/**Holds the linear position in world space. */
		Vector3 mPosition;

		/**Holds the linear velocity in world space. */
		Vector3 mVelocity;

		/**Holds the linear acceleration in world space. */
		Vector3 mAcceleration;

		/**Holds the amount of damping applied to linear motion. */
		Precision mDamping;

		/**Holds the inverse of the mass of the particle. */
		Precision mInverseMass;

		/** *Holds the accumulated force to be applied at the start of the next frame. */
		Vector3 mAccumulatedForce;

		/**Holds Acceleration of gravity on the particle. */
		Vector3 mGravity;

		/**Function that clears the accumulator */
		void ClearAccumulator();
	};
}