#pragma once
#include "Vector3.h"
#include <cstdint>

namespace PolishPhysics
{
	struct CollisionData;
	class CollisionSphere;
	class CollisionPlane;
	class CollisionBox;

	class CollisionDetector
	{
	public:

		static std::uint32_t SphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane,
			CollisionData *data);

		static std::uint32_t SphereAndTruePlane(const CollisionSphere &sphere, const CollisionPlane &plane,
			CollisionData *data);

		static std::uint32_t SphereAndSphere(const CollisionSphere &one, const CollisionSphere &two,
			CollisionData *data);

		static std::uint32_t BoxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane,
			CollisionData *data);

		static std::uint32_t BoxAndBox(const CollisionBox &one, const CollisionBox &two, CollisionData *data);

		static std::uint32_t BoxAndPoint(const CollisionBox &box, const Vector3 &point, CollisionData *data);

		static std::uint32_t BoxAndSphere(const CollisionBox &box, const CollisionSphere &sphere,
			CollisionData *data);

		static Precision PenetrationOnAxis(const CollisionBox &one, const CollisionBox &two, const Vector3 &axis,
			const Vector3 &toCentre);

		static bool TryAxis(const CollisionBox &one, const CollisionBox &two, Vector3 axis,
			const Vector3& toCentre, unsigned index, Precision& smallestPenetration, uint32_t &smallestCase);

		static void FillPointFaceBoxBox(const CollisionBox &one, const CollisionBox &two, const Vector3 &toCentre,
			CollisionData *data, uint32_t best, Precision pen);
	};
}