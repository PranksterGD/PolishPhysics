#include "pch.h"

using namespace std;
using namespace PolishPhysics;

ParticleBuoyancyForceGenerator::ParticleBuoyancyForceGenerator(Precision maxDepth, Precision volume, Precision liquidHeight, Precision liquidDensity /* = 1000.0f */) :
	mMaxDepth(maxDepth), mVolume(volume), mLiquidHeight(liquidHeight), mLiquidDensity(liquidDensity)
{

}

void ParticleBuoyancyForceGenerator::UpdateForce(class Particle& particle, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	Precision depth = particle.GetPosition().Y;

	//Check if we're out of the liquid.
	if (depth < mLiquidHeight + mMaxDepth)
	{
		Vector3 force;

		//Check if we're at max depth.
		if (depth <= mLiquidHeight - mMaxDepth)
		{
			force.Y = mLiquidDensity * mVolume;
			particle.AddForce(force);
		}
		else
		{
			force.Y = mLiquidDensity * mVolume * (depth - mMaxDepth - mLiquidHeight) / 2 * mMaxDepth;
			particle.AddForce(force);
		}
	}
}

void ParticleAnchoredBungeeForceGenerator::SetAnchorPosition(const Vector3& anchor)
{
	mAnchor = anchor;
}

Vector3 ParticleAnchoredBungeeForceGenerator::GetAnchorPosition() const
{
	return mAnchor;
}