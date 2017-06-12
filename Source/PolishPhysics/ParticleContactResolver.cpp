#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleContactResolver::ParticleContactResolver(std::uint32_t iterations):
	mIterations(iterations), mIterationsUsed(0)
{

}

void ParticleContactResolver::SetMaxIterations(std::uint32_t iterations)
{
	mIterations = iterations;
}

Precision ParticleContactResolver::GetMaxIterations()
{
	return static_cast<Precision> (mIterations);
}

void ParticleContactResolver::ResolveContacts(ParticleContact* contactArray, std::uint32_t numContacts, Precision deltaTime)
{
	uint32_t i;

	mIterationsUsed = 0;

	while (mIterationsUsed < mIterations)
	{
		Precision max = PRECISION_MAX;

		uint32_t maxIndex = numContacts;

		for (i = 0; i < numContacts; ++i)
		{
			Precision sperationVelocity = contactArray[i].GetSeperatingVelocity();

			if (sperationVelocity < max &&
				sperationVelocity < 0.0f || contactArray[i].mPenetration > 0.0f)
			{
				max = sperationVelocity;
				maxIndex = i;
			}
		}

		if (maxIndex == numContacts)
		{
			break;
		}

		contactArray[maxIndex].Resolve(deltaTime);
		++mIterationsUsed;
	}
}