#pragma once
#include "Vector3.h"

namespace PolishPhysics
{
	struct BoundingSphere
	{
		Vector3 mCenter;

		Precision mRadius;

	public:

		/**Creates a new bounding sphere at the given center and radius
		* @param center- The center of  the sphere.
		* @param radius- The radius of the sphere.*/
		BoundingSphere(const Vector3& center, Precision radius);

		/**Creates a bounding sphere to enclose the two given bounding spheres.
		* @param first- One of the two spheres to enclose.
		* @param second- The other sphere to enclose.*/
		BoundingSphere(const BoundingSphere& first, const BoundingSphere& second);

		/**Checks if the bounding sphere overlaps the other given sphere.
		* @param other - The other sphere to check against.*/
		bool Overlaps(const BoundingSphere& other) const;

		/**Returns the volume of this bounding volume.*/
		Precision GetSize() const;

		/** Reports how much this bounding sphere would have to grow by to incorporate
		the given bounding sphere.
		* @param other- The other bounding sphere to incorporate. */
		Precision GetGrowth(const BoundingSphere &other) const;
	};
}