#pragma once

namespace PolishPhysics
{
	struct PotentialContact
	{
		// Holds the bodies that might be in contact
		class RigidBody* mbodies[2];
	};
}