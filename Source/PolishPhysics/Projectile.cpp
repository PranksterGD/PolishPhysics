#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Projectile::Projectile() :
	mExpired(false), mType(Projectile::ShotType::INVALID)
{

}

void Projectile::SetType(Projectile::ShotType type)
{
	mType = type;
}


Projectile::ShotType Projectile::GetType() const
{
	return mType;
}

Vector3 Projectile::GetPosition() const
{
	return mParticle.GetPosition();
}

void Projectile::Fire()
{
	if (mType != Projectile::ShotType::INVALID)
	{
		switch (mType)
		{
		case PolishPhysics::Projectile::ShotType::PISTOL:
			mParticle.SetMass(2.0f);
			mParticle.SetVelocity(Vector3(0.0f, 0.0f, 35.0f)); // 35m/s
			mParticle.SetAcceleration(Vector3(0.0f, -1.0f, 0.0f));
			mParticle.SetDamping(0.99f);
			break;
		case PolishPhysics::Projectile::ShotType::CROSSBOW:
			mParticle.SetMass(1.0f);
			mParticle.SetVelocity(Vector3(0.0f, 0.0f, 20.0f)); // 35m/s
			mParticle.SetAcceleration(Vector3(0.0f, -5.0f, 0.0f));
			mParticle.SetDamping(0.8f);
			break;
		}

		mParticle.SetPosition(Vector3(0.0f, 100.0f, 0.0f));

		//mParticle.ClearAccumulator();
	}

}

bool Projectile::IsExpired() const
{
	return mExpired;
}

void Projectile::Update()
{
	mParticle.Integrate(0.167f);

	//Actual collision detection required.
	if (mParticle.GetPosition().Y < -50.0f || mParticle.GetPosition().Z > 200.0f)
	{
		mExpired = true;
	}
}