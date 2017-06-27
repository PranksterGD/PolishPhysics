#pragma once
#include "Primitive.h"

namespace PolishPhysics
{
	class CollisionBox : public Primitive
	{
	public:

		/**Holds the half-sizes of the box along each of its local axes.*/
		Vector3 halfSize;
	};

}