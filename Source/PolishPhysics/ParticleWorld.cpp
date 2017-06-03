#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations/* =0 */) :
	mResolver(iterations), mMaxContacts(maxContacts)
{
	mContacts = new ParticleContact[mMaxContacts];

	mShouldCalculateIterations = iterations == 0;
}

ParticleWorld::~ParticleWorld()
{
	delete[] mContacts;
}

void ParticleWorld::StartFrame()
{
	for (auto it = mParticles.begin(); it != mParticles.end(); ++it)
	{
		(*it)->ClearAccumulator();
	}
}

uint32_t ParticleWorld::GenerateContacts()
{
	uint32_t limit = mMaxContacts;
	ParticleContact* nextContact = mContacts;

	for (auto it = mContactGenerators.begin(); it != mContactGenerators.end(); ++it)
	{
		uint32_t contactsGenerated = (*it)->AddContact(nextContact, limit);
		limit -= contactsGenerated;
		nextContact += contactsGenerated;

		if (limit <= 0)
		{
			break;
		}
	}

	return mMaxContacts - limit;
}

void ParticleWorld::Integrate(Precision deltaTime)
{
	for (auto it = mParticles.begin(); it != mParticles.end(); ++it)
	{
		(*it)->Integrate(deltaTime);
	}
}

void ParticleWorld::Update(Precision deltaTime)
{
	mRegistry.UpdateForces(deltaTime);

	Integrate(deltaTime);

	uint32_t generatedContacts = GenerateContacts();

	if (generatedContacts > 0)
	{
		if (mShouldCalculateIterations)
		{
			mResolver.SetMaxIterations(generatedContacts * 2);
		}

		mResolver.ResolveContacts(mContacts, generatedContacts, deltaTime);
	}
}