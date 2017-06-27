#pragma once

namespace PolishPhysics
{
	class CollisionSphere;
	class CollisionPlane;
	class CollisionBox;

	class IntersectionTests
	{
	public:

		static bool SphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane);

		static bool SphereAndSphere(const CollisionSphere &one, const CollisionSphere &two);

		static bool BoxAndBox(const CollisionBox &one, const CollisionBox &two);

		static bool BoxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane);

		static Precision TransformToAxis(const CollisionBox &box, const Vector3 &axis);

		/** This function checks if the two boxes overlap along the given axis. The final parameter toCentre
		* is used to pass in the vector between the boxes centre points, to avoid having to
		* recalculate it each time.*/
		static bool OverlapOnAxis(const CollisionBox &one, const CollisionBox &two, const Vector3 &axis,
			const Vector3 &toCentre);
	};

}