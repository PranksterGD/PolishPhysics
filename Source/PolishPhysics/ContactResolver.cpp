#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ContactResolver::ContactResolver(uint32_t iterations, Precision velocityEpsilon,
	Precision positionEpsilon)
{
	mVelocityIterations = iterations;
	mPositionIterations = iterations;

	mVelocityEpsilon = velocityEpsilon;
	mPositionEpsilon = positionEpsilon;
}

void ContactResolver::ResolveContacts(class Contact* contactArray, unsigned numContacts, Precision deltaTime)
{
	if (numContacts == 0)
	{
		return;
	}

	PrepareContacts(contactArray, numContacts, deltaTime);
	AdjustPositions(contactArray, numContacts, deltaTime);
	AdjustVelocities(contactArray, numContacts, deltaTime);
}

void ContactResolver::PrepareContacts(class Contact *contactArray, unsigned numContacts, Precision deltaTime)
{
	Contact* lastContact = contactArray + numContacts;

	for (Contact* contact = contactArray; contact < lastContact; ++contact)
	{
		contact->CaluclateInternals(deltaTime);
	}
}

void ContactResolver::AdjustPositions(class Contact *contacts, unsigned numContacts, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	uint32_t i, index;
	Vector3 linearChange[2], angularChange[2];
	Precision max;
	Vector3 deltaPosition;

	mPositionIterationsUsed = 0;

	while (mPositionIterationsUsed < mPositionIterations)
	{
		max = mPositionEpsilon;
		index = numContacts;

		for (i = 0; i < numContacts; ++i)
		{
			if (contacts[i].mPenetration > max)
			{
				max = contacts[i].mPenetration;
				index = i;
			}
		}

		if (index == numContacts)
		{
			break;
		}

		//Wake up the bodies that need to be woken
		contacts[index].MatchAwakeState();

		//Resolve the penetration.
		contacts[index].ApplyPositionChange(linearChange, angularChange, max);

		//This action may have changed the penetration of other bodies, so we update related contacts.

		for (i = 0; i < numContacts; ++i)
		{
			//Check each body of the contact
			for (uint32_t j = 0; j < 2; ++j)
			{
				if (contacts[i].mBodies[j] != nullptr)
				{
					//Check for a match with one of the bodies in the resolved contact
					for (uint32_t k = 0; k < 2; ++k)
					{
						if (contacts[i].mBodies[j] == contacts[index].mBodies[k])
						{
							deltaPosition = linearChange[k] + angularChange[k].VectorProduct(contacts[i].mRelativeContactPosition[j]);

							//The sign of the change is positive if we're dealing with the second body in a contact, negative otherwise
							contacts[i].mPenetration += deltaPosition.ScalarProduct(contacts[i].mContactNormal) * (j ? 1 : -1);
						}
					}
				}
			}
		}

		mPositionIterationsUsed++;
	}
}

void ContactResolver::AdjustVelocities(class Contact *contactArray, unsigned numContacts, Precision deltaTime)
{
	Vector3 velocityChange[2], rotationChange[2];
	Vector3 deltaVel;

	// iteratively handle impacts in order of severity.
	mVelocityIterationsUsed = 0;
	while (mVelocityIterationsUsed < mVelocityIterations)
	{
		// Find contact with maximum magnitude of probable velocity change.
		Precision max = mVelocityEpsilon;

		uint32_t index = numContacts;
		for (unsigned i = 0; i < numContacts; i++)
		{
			if (contactArray[i].mDesiredDeltaVelocity > max)
			{
				max = contactArray[i].mDesiredDeltaVelocity;
				index = i;
			}
		}
		if (index == numContacts)
		{
			break;
		}

		// Match the awake state at the contact
		contactArray[index].MatchAwakeState();

		// Do the resolution on the contact that came out top.
		contactArray[index].ApplyVelocityChange(velocityChange, rotationChange);

		// With the change in velocity of the two bodies, the update of
		// contact velocities means that some of the relative closing
		// velocities need recomputing.
		for (unsigned i = 0; i < numContacts; i++)
		{
			// Check each body in the contact
			for (unsigned b = 0; b < 2; b++)
			{
				if (contactArray[i].mBodies[b])
				{
					// Check for a match with each body in the newly
					// resolved contact
					for (unsigned d = 0; d < 2; d++)
					{
						if (contactArray[i].mBodies[b] == contactArray[index].mBodies[d])
						{
							deltaVel = velocityChange[d] + rotationChange[d].VectorProduct(contactArray[i].mRelativeContactPosition[b]);

							// The sign of the change is negative if we're dealing
							// with the second body in a contact.
							contactArray[i].mContactVelocity += contactArray[i].mContactToWorld.TransformTranspose(deltaVel) * (b ? -1.0f : 1.0f);
							contactArray[i].CalculateDeltaVelocity(deltaTime);
						}
					}
				}
			}
		}
		mVelocityIterationsUsed++;
	}
}