#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Precision IntersectionTests::TransformToAxis(const CollisionBox &box, const Vector3 &axis)
{
	return
		box.halfSize.X * precision_abs(axis.ScalarProduct(box.GetAxis(0))) +
		box.halfSize.Y * precision_abs(axis.ScalarProduct(box.GetAxis(1))) +
		box.halfSize.Z * precision_abs(axis.ScalarProduct(box.GetAxis(2)));
}

bool IntersectionTests::OverlapOnAxis(const CollisionBox &one, const CollisionBox &two, const Vector3 &axis,
	const Vector3 &toCentre)
{
	// Project the half-size of one onto axis
	Precision oneProject = TransformToAxis(one, axis);
	Precision twoProject = TransformToAxis(two, axis);

	// Project this onto the axis
	Precision distance = precision_abs(toCentre.ScalarProduct(axis));

	bool returnValue = (distance < oneProject + twoProject);

	// Check for overlap
	return returnValue;
}

bool IntersectionTests::SphereAndSphere(const CollisionSphere &one, const CollisionSphere &two)
{
	// Find the vector between the objects
	Vector3 midPoint = one.GetAxis(3) - two.GetAxis(3);

	// See if it is large enough.
	return midPoint.SquareMagnitude() <
		(one.mRadius + two.mRadius)*(one.mRadius + two.mRadius);
}

bool IntersectionTests::SphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane)
{
	// Find the distance from the origin
	Precision distance = plane.mNormal.ScalarProduct(sphere.GetAxis(3)) - sphere.mRadius;

	// Check for the intersection
	return distance <= plane.mOffset;
}

bool IntersectionTests::BoxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane)
{
	// Work out the projected radius of the box onto the plane direction
	Precision projectedRadius = TransformToAxis(box, plane.mNormal);

	// Work out how far the box is from the origin
	Precision boxDistance = plane.mNormal.ScalarProduct(box.GetAxis(3)) - projectedRadius;

	// Check for the intersection
	return boxDistance <= plane.mOffset;
}

#define TEST_OVERLAP(axis) OverlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::BoxAndBox(const CollisionBox &one, const CollisionBox &two)
{
	// Find the vector between the two centers

	Vector3 twoPos = two.GetAxis(3);
	Vector3 onePos = one.GetAxis(3);
	Vector3 toCentre = two.GetAxis(3) - one.GetAxis(3);

	return (
		// Check on box one's axes first
		TEST_OVERLAP(one.GetAxis(0)) &&
		TEST_OVERLAP(one.GetAxis(1)) &&
		TEST_OVERLAP(one.GetAxis(2)) &&

		// And on two's
		TEST_OVERLAP(two.GetAxis(0)) &&
		TEST_OVERLAP(two.GetAxis(1)) &&
		TEST_OVERLAP(two.GetAxis(2)) &&

		// Now on the cross products
		TEST_OVERLAP(one.GetAxis(0).VectorProduct(two.GetAxis(0))) &&
		TEST_OVERLAP(one.GetAxis(0).VectorProduct(two.GetAxis(1))) &&
		TEST_OVERLAP(one.GetAxis(0).VectorProduct(two.GetAxis(2))) &&
		TEST_OVERLAP(one.GetAxis(1).VectorProduct(two.GetAxis(0))) &&
		TEST_OVERLAP(one.GetAxis(1).VectorProduct(two.GetAxis(1))) &&
		TEST_OVERLAP(one.GetAxis(1).VectorProduct(two.GetAxis(2))) &&
		TEST_OVERLAP(one.GetAxis(2).VectorProduct(two.GetAxis(0))) &&
		TEST_OVERLAP(one.GetAxis(2).VectorProduct(two.GetAxis(1))) &&
		TEST_OVERLAP(one.GetAxis(2).VectorProduct(two.GetAxis(2)))
		);
}

#undef TEST_OVERLAP