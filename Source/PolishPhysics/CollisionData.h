#pragma once
#include "Contact.h"
#include <cstdint>

namespace PolishPhysics
{
	struct CollisionData
	{

		/** Holds the first contact in the array.*/
		Contact *mFirstContact;

		/**Holds the contact array to write into. */
		Contact* mContacts;

		/**Holds the maximum number of contacts the array can use. */
		std::uint32_t mContactsLeft;

		std::uint32_t mContactsFound;

		/** Holds the friction value to write into any collisions. */
		Precision mFriction;

		/** Holds the restitution value to write into any collisions. */
		Precision mRestitution;

		/**
		* Holds the collision tolerance, even uncolliding objects this
		* close should have collisions generated.
		*/
		Precision mTolerance;

		/** Checks if there are more contacts available in the contact data.
		* @return A boolean- True if more contacts are available, false otherwise.*/
		bool HasMoreContacts() const;

		/** Resets the data so that it has no used contacts recorded.
		* @param maxContacts - The max contacts of the new collision data.*/
		void Reset(std::uint32_t maxContacts);

		/** Notifies the data that the given number of contacts have been added.
		* @param count - The number of contacts that have been added.*/
		void AddContacts(std::uint32_t count);
	};
}