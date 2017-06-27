#pragma once
#include "RigidBody.h"
#include "Matrix4.h"

namespace PolishPhysics
{
	class Primitive
	{
		friend class IntersectionTests;
		friend class CollisionDetector;

	public:

		RigidBody* mBody;

		Matrix4 mOffset;

		bool mHasOffset;

	protected:
		/** The resultant transform of the primitive. This is calculated by combining the offset of
		the primitive with the transform of the rigid body.*/
		Matrix4 mTransform;

	public:

		Primitive();

		/** Calculates the internals for the primitive. */
		void CalculateInternals();

		/** This is a convenience function to allow access to the  axis vectors in the transform 
		for this primitive.*/
		Vector3 GetAxis(std::uint32_t index) const;

		/** Returns the resultant transform of the primitive, calculated from  the combined
		offset of the primitive and the transform of the rigid body to which it is attached.*/
		const Matrix4& GetTransform() const;
	};
}