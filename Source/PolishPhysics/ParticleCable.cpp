#include "pch.h"

using namespace std;
using namespace PolishPhysics;


uint32_t ParticleCable::AddContact(class ParticleContact *contact, std::uint32_t limit) const
{
	Precision currentLength = GetCurrentLength();

	uint32_t contactsCreated = 0;

	if (currentLength > mMaxLength)
	{
		contact->mParticles[0] = mParticles[0];
		contact->mParticles[1] = mParticles[1];

		Vector3 normal = mParticles[1]->GetPosition() - mParticles[0]->GetPosition();
		normal.Normalize();

		contact->mContactNormal = normal;

		contact->mPenetration = currentLength - mMaxLength;
		contact->mRestitution = mRestitution;

		contactsCreated = 1;
	}

	return contactsCreated;
}