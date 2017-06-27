#include "pch.h"

using namespace std;
using namespace PolishPhysics;

void Contact::setBodyData(RigidBody* one, RigidBody *two, Precision friction, Precision restitution)
{
	mBodies[0] = one;
	mBodies[1] = two;
	mFriction = friction;
	mRestitution = restitution;
}