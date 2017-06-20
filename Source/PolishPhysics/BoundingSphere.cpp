#include "pch.h"

using namespace std;
using namespace PolishPhysics;

BoundingSphere::BoundingSphere(const Vector3& center, Precision radius) :
	mCenter(center), mRadius(radius)
{

}

BoundingSphere::BoundingSphere(const BoundingSphere& first, const BoundingSphere& second)
{
	Vector3 centerOffset = second.mCenter - first.mCenter;

	Precision distance = centerOffset.SquareMagnitude();
	Precision radiusDiff = second.mRadius - first.mRadius;

	//Check if the larger sphere encloses the smaller one.
	if (radiusDiff*radiusDiff >= distance)
	{
		if (first.mRadius > second.mRadius)
		{
			mCenter = first.mCenter;
			mRadius = first.mRadius;
		}
		else
		{
			mCenter = second.mCenter;
			mRadius = second.mRadius;
		}
	}
	//Otherwise, need to work with partially overlapping spheres
	else
	{
		distance = precision_sqrt(distance);

		mRadius = (distance + first.mRadius + second.mRadius) *  static_cast<Precision>(0.5);

		mCenter = first.mCenter;

		if (distance > 0)
		{
			mCenter += centerOffset *((mRadius - first.mRadius) / distance);
		}
	}
}

bool BoundingSphere::Overlaps(const BoundingSphere& other) const
{
	Precision distanceSquared = (mCenter - other.mCenter).SquareMagnitude();

	return distanceSquared < (mRadius + other.mRadius) * (mRadius + other.mRadius);
}

Precision BoundingSphere::GetSize() const
{
	return (static_cast<Precision>(1.333333)) * static_cast<Precision>(3.14) * mRadius * mRadius * mRadius;
}

Precision BoundingSphere::GetGrowth(const BoundingSphere &other) const
{
	BoundingSphere newSphere(*this, other);

	// We return a value proportional to the change in surface
	// area of the sphere.
	return newSphere.mRadius*newSphere.mRadius - mRadius*mRadius;
}
