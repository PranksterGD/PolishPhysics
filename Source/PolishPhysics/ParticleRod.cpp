#include "pch.h"

using namespace std;
using namespace PolishPhysics;

uint32_t ParticleRod::AddContact(class ParticleContact *contact, std::uint32_t limit) const
{
	UNREFERENCED_PARAMETER(limit);
	Precision currentLength = GetCurrentLength();

	uint32_t contactsCreated = 0;

	if (currentLength != mLength)
	{
		contact->mParticles[0] = mParticles[0];
		contact->mParticles[1] = mParticles[1];

		Vector3 normal = mParticles[1]->GetPosition() - mParticles[0]->GetPosition();
		normal.Normalize();

		if (currentLength > mLength)
		{
			contact->mContactNormal = normal;
			contact->mPenetration = currentLength - mLength;
		}
		else
		{
			contact->mContactNormal = normal * -1;
			contact->mPenetration = mLength - currentLength;
		}

		contact->mRestitution = 0.0f;

		contactsCreated = 1;
	}

	return contactsCreated;
}