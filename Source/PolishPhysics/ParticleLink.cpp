#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Precision ParticleLink::GetCurrentLength() const
{
	Vector3 relativePosition = mParticles[0]->GetPosition() - mParticles[1]->GetPosition();
	
	return relativePosition.Magnitude();
}