#pragma once
#include "Vector3.h"
#include "Matrix3.h"
#include "RigidBody.h"

namespace PolishPhysics
{
	/**A contact represents two bodies in contact. */
	class Contact
	{
		friend class CollisionDetector;
		friend class ContactResolver;

	public:

		RigidBody* mBodies[2];

		Vector3 mContactPosition;

		Vector3 mContactNormal;

		Precision mPenetration;

		Precision mRestitution;

		Precision mFriction;

		/**Sets the data that doesn't normally depend on the position
		* of the contact (i.e. the bodies, and their material properties). */
		void setBodyData(RigidBody* one, RigidBody *two, Precision friction, Precision restitution);


	protected:

		//Holds the closing velocity at the point of contact.
		Vector3 mContactVelocity;

		//Holds the world space coordinates of the contact point in relation to the position of the two objects
		Vector3 mRelativeContactPosition[2];

		//A transform matrix that converts cooridnates from world space to contact space.
		Matrix3 mContactToWorld;

		//Holds the required change in velocity for this contact to be resolved.
		Precision mDesiredDeltaVelocity;

		/**Calculates an orthonormal basis for the contact point.*/
		void CalculateContactBasis();

		Vector3 CalculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor);

		void CaluclateInternals(Precision deltaTime);

		void SwapBodies();

		Vector3 CalculateLocalVelocity(std::uint32_t bodyIndex, Precision deltaTime);

		void CalculateDeltaVelocity(Precision deltaTime);

		void MatchAwakeState();

		void ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], Precision penetration);

		void ApplyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);
	};
}