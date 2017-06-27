#pragma once
#include "Primitive.h"

namespace PolishPhysics
{
	class CollisionPlane : public Primitive
	{
	public:

		Vector3 mNormal;
		Precision mOffset;
	};

}