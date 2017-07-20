#include "pch.h"

using namespace std;
using namespace PolishPhysics;

RigidBodyWorld::RigidBodyWorld() 
{

}

void RigidBodyWorld::AddBody(RigidBody& body)
{
	mBodies.push_back(&body);
}

void RigidBodyWorld::RegisterForce(RigidBody& body, RigidBodyForceGenerator& generator)
{
	mRegistry.Add(body, generator);
}

RigidBodyWorld::~RigidBodyWorld()
{

}

void RigidBodyWorld::StartFrame()
{
	for (auto it = mBodies.begin(); it != mBodies.end(); ++it)
	{
		(*it)->ClearAccumulators();
		(*it)->CalculateDerivedData();
	}
}

void RigidBodyWorld::Integrate(Precision deltaTime)
{
	for (auto it = mBodies.begin(); it != mBodies.end(); ++it)
	{
		(*it)->Integrate(deltaTime);
	}
}

void RigidBodyWorld::Update(Precision deltaTime)
{
	mRegistry.UpdateForces(deltaTime);

	Integrate(deltaTime);
}
