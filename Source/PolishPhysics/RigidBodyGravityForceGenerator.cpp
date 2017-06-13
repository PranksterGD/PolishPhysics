#include "pch.h"

using namespace std;
using namespace PolishPhysics;

RigidBodyGravityForceGenerator::RigidBodyGravityForceGenerator(const Vector3& gravity) :
	mGravity(gravity)
{

}

void RigidBodyGravityForceGenerator::UpdateForce(class RigidBody& body, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	if (!body.HasInfiniteMass())
	{
		body.AddForce(mGravity * body.GetMass());
	}
}