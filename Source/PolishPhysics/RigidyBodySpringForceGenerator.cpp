#include "pch.h"

using namespace std;
using namespace PolishPhysics;

RigidBodySpringForceGenerator::RigidBodySpringForceGenerator(const RigidBody& otherBody, Precision springConstant, Precision restLength,
	const Vector3& connectionPoint, const Vector3& otherConnectionPoint) :
	mOtherBody(&otherBody), mSpringConstant(springConstant), mRestLength(restLength), mConnectionPoint(connectionPoint),
	mOtherConnectionPoint(otherConnectionPoint)
{

}

void RigidBodySpringForceGenerator::UpdateForce(class RigidBody& body, Precision deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	Vector3 lws = body.GetPointInWorldSpace(mConnectionPoint);
	Vector3 ows = mOtherBody->GetPointInWorldSpace(mOtherConnectionPoint);

	Vector3 force = lws - ows;

	Precision magnitude = force.Magnitude();
	magnitude = precision_abs(magnitude - mRestLength);
	magnitude *= mSpringConstant;

	force.Normalize();
	force *= -magnitude;

	body.AddForceAtPoint(force, lws);
}