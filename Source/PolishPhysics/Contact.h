#pragma once
#include "Vector3.h"

namespace PolishPhysics
{
	/**A contact represents two bodies in contact. */
	class Contact
	{
		friend class CollisionDetector;

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
	};
}