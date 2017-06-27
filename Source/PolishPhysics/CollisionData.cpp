#include "pch.h"

using namespace std;
using namespace PolishPhysics;

bool CollisionData::HasMoreContacts() const
{
	return mContactsLeft > 0;
}

void CollisionData::Reset(std::uint32_t maxContacts)
{
	mContactsLeft = maxContacts;
	mContactsFound = 0;
	mContacts = mFirstContact;
}

void CollisionData::AddContacts(std::uint32_t count)
{
	// Reduce the number of contacts remaining, add number used
	mContactsLeft -= count;
	mContactsFound += count;

	// Move the array forward
	mContacts += count;
}

