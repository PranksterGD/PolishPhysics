#include "pch.h"

using namespace std;
using namespace PolishPhysics;

/*
* This function checks if the two boxes overlap
* along the given axis, returning the ammount of overlap.
* The final parameter toCentre
* is used to pass in the vector between the boxes centre
* points, to avoid having to recalculate it each time.

*/
Precision CollisionDetector::PenetrationOnAxis(const CollisionBox &one, const CollisionBox &two, const Vector3 &axis,
	const Vector3 &toCentre)
{
	// Project the half-size of one onto axis
	Precision oneProject = IntersectionTests::TransformToAxis(one, axis);
	Precision twoProject = IntersectionTests::TransformToAxis(two, axis);

	// Project this onto the axis
	Precision distance = precision_abs(toCentre.ScalarProduct(axis));

	// Return the overlap (i.e. positive indicates
	// overlap, negative indicates separation).
	return oneProject + twoProject - distance;
}


bool CollisionDetector::TryAxis(const CollisionBox &one, const CollisionBox &two, Vector3 axis,
	const Vector3& toCentre, unsigned index, Precision& smallestPenetration, uint32_t &smallestCase)
{
	// Make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.SquareMagnitude() < 0.0001) return true;
	axis.Normalize();

	Precision penetration = PenetrationOnAxis(one, two, axis, toCentre);

	if (penetration < 0)
	{
		return false;
	}

	if (penetration < smallestPenetration) 
	{
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

void CollisionDetector::FillPointFaceBoxBox(const CollisionBox &one, const CollisionBox &two, const Vector3 &toCentre,
	CollisionData *data, uint32_t best, Precision pen)
{
	// This method is called when we know that a vertex from
	// box two is in contact with box one.

	Contact* contact = data->mContacts;

	// We know which axis the collision is on (i.e. best),
	// but we need to work out which of the two faces on
	// this axis.
	Vector3 normal = one.GetAxis(best);
	if (one.GetAxis(best).ScalarProduct(toCentre) > 0)
	{
		normal = normal * -1.0f;
	}

	// Work out which vertex of box two we're colliding with.
	// Using toCentre doesn't work!
	Vector3 vertex = two.halfSize;
	if (two.GetAxis(0).ScalarProduct(normal) < 0) vertex.X = -vertex.X;
	if (two.GetAxis(1).ScalarProduct(normal) < 0) vertex.Y = -vertex.Y;
	if (two.GetAxis(2).ScalarProduct(normal) < 0) vertex.Z = -vertex.Z;

	// Create the contact data
	contact->mContactNormal = normal;
	contact->mPenetration = pen;
	contact->mContactPosition = two.GetTransform() * vertex;
	contact->setBodyData(one.mBody, two.mBody,
		data->mFriction, data->mRestitution);
}

static inline Vector3 ContactPoint(const Vector3 &pOne, const Vector3 &dOne, Precision oneSize,
	const Vector3 &pTwo, const Vector3 &dTwo, Precision twoSize, bool useOne)
{
	Vector3 toSt, cOne, cTwo;
	Precision dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	Precision denom, mua, mub;

	smOne = dOne.SquareMagnitude();
	smTwo = dTwo.SquareMagnitude();
	dpOneTwo = dTwo.ScalarProduct(dOne);

	toSt = pOne - pTwo;
	dpStaOne = dOne.ScalarProduct(toSt);
	dpStaTwo = dTwo.ScalarProduct(toSt);

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parrallel lines
	if (precision_abs(denom) < 0.0001f) 
	{
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	// If either of the edges has the nearest point out
	// of bounds, then the edges aren't crossed, we have
	// an edge-face contact. Our point is on the edge, which
	// we know from the useOne parameter.
	if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}

uint32_t CollisionDetector::SphereAndSphere(const CollisionSphere &one, const CollisionSphere &two, CollisionData *data)
{
	//Make sure we have contact to write to.
	if (data->mContactsLeft <= 0)
	{
		return 0;
	}

	Vector3 positionOne = one.GetAxis(3);
	Vector3 positionTwo = two.GetAxis(3);

	Vector3 midPoint = positionOne - positionTwo;
	Precision size = midPoint.Magnitude();

	//Early out if the midpoint lies outside the radius of both the spheres.
	if (size <= 0.0f || size >= one.mRadius + two.mRadius)
	{
		return 0;
	}

	Vector3 normal = midPoint* (static_cast<Precision> (1.0f) / size);

	Contact* contact = data->mContacts;
	contact->mContactNormal = normal;
	contact->mContactPosition = positionOne + midPoint * static_cast<Precision> (0.5f);
	contact->mPenetration = (one.mRadius + two.mRadius - size);
	contact->setBodyData(one.mBody, two.mBody, data->mFriction, data->mRestitution);

	data->AddContacts(1);
	return 1;
}

uint32_t CollisionDetector::SphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data)
{
	//Make sure we have contact to write to.
	if (data->mContactsLeft <= 0)
	{
		return 0;
	}

	Vector3 position = sphere.GetAxis(3);

	Precision distance = plane.mNormal.ScalarProduct(position);
	distance -= sphere.mRadius;
	distance -=plane.mOffset;

	//Early out if the sphere's position is outside the plane.
	if (distance >= 0.0f)
	{
		return 0;
	}

	Contact* contact = data->mContacts;
	contact->mContactNormal = plane.mNormal;
	contact->mPenetration = distance * -1.0f;
	contact->mContactPosition = position - plane.mNormal* (distance + sphere.mRadius);
	contact->setBodyData(sphere.mBody, nullptr, data->mFriction, data->mRestitution);

	data->AddContacts(1);
	return 1;
}

uint32_t CollisionDetector::SphereAndTruePlane(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data)
{	
	//Make sure we have contact to write to.
	if (data->mContactsLeft <= 0)
	{
		return 0;
	}

	Vector3 position = sphere.GetAxis(3);

	Precision centreDistance = plane.mNormal.ScalarProduct(position) - plane.mOffset;

	//Check if we're within the radius
	if (centreDistance* centreDistance > sphere.mRadius* sphere.mRadius)
	{
		return 0;
	}

	//Check which side of the plane we're on.
	Vector3 normal = plane.mNormal;
	Precision penetration = -centreDistance;

	if (centreDistance < 0)
	{
		normal *= -1;
		penetration = -penetration;
	}

	penetration += sphere.mRadius;

	Contact* contact = data->mContacts;
	contact->mContactNormal = normal;
	contact->mPenetration = penetration;
	contact->mContactPosition = position - plane.mNormal * centreDistance;
	contact->setBodyData(sphere.mBody, nullptr, data->mFriction, data->mRestitution);

	data->AddContacts(1);
	return 1;
}

uint32_t CollisionDetector::BoxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane, CollisionData *data)
{
	if (data->mContactsLeft <= 0)
	{
		return 0;
	}

	if (!IntersectionTests::BoxAndHalfSpace(box, plane))
	{
		return 0;
	}

	//Each combination of + and - for each of the halfsizes
	static Precision mults[8][3] = 
	{
		{1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}
	};

	Contact* contact = data->mContacts;
	uint32_t contactsGenerated = 0;

	for (uint32_t i = 0; i < 8; ++i)
	{
		//Calculate the position of each vertex.
		Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
		vertexPos.ComponentProductAssignment(box.halfSize);
		vertexPos = box.mTransform.Transform(vertexPos);

		//Calculate distance from the plane
		Precision vertexDistance = vertexPos.ScalarProduct(plane.mNormal);

		//Compare this to the plane's distance
		if (vertexDistance <= plane.mOffset)
		{
			//The contact point is halfway between the vertex and the plane.
			contact->mContactPosition = plane.mNormal;
			contact->mContactPosition *= (vertexDistance - plane.mOffset);
			contact->mContactPosition += vertexPos;
			contact->mContactNormal = plane.mNormal;
			contact->mPenetration = plane.mOffset - vertexDistance;
			contact->setBodyData(box.mBody, nullptr, data->mFriction, data->mRestitution);

			++contact;
			++contactsGenerated;

			if (contactsGenerated == data->mContactsLeft)
			{
				return contactsGenerated;
			}
		}
	}

	data->AddContacts(contactsGenerated);
	return contactsGenerated;
}

uint32_t CollisionDetector::BoxAndSphere(const CollisionBox &box, const CollisionSphere &sphere, CollisionData *data)
{
	//Transform the center of the sphere into box coordinates.
	Vector3 center = sphere.GetAxis(3);
	Vector3 relCenter = box.GetTransform().TransformInverse(center);

	//Early out if the center is not within the box
	if (precision_abs(relCenter.X) - sphere.mRadius > box.halfSize.X ||
		precision_abs(relCenter.Y) - sphere.mRadius > box.halfSize.Y ||
		precision_abs(relCenter.Z) - sphere.mRadius > box.halfSize.Z)
	{
		return 0;
	}

	Vector3 closestPt;
	Precision distance;

	//Clamp each coordinate to the box.
	distance = relCenter.X;
	if (distance > box.halfSize.X)
	{
		distance = box.halfSize.X;
	}
	if (distance < -box.halfSize.X)
	{
		distance = -box.halfSize.X;
	}
	closestPt.X = distance;

	distance = relCenter.Y;
	if (distance > box.halfSize.Y)
	{
		distance = box.halfSize.Y;
	}
	if (distance < -box.halfSize.Y)
	{
		distance = -box.halfSize.Y;
	}
	closestPt.Y = distance;

	distance = relCenter.Z;
	if (distance > box.halfSize.Z)
	{
		distance = box.halfSize.Z;
	}
	if (distance < -box.halfSize.Z)
	{
		distance = -box.halfSize.Z;
	}
	closestPt.Z = distance;

	//Check to see if we're in contact

	distance = (closestPt - relCenter).SquareMagnitude();

	if (distance > sphere.mRadius* sphere.mRadius)
	{
		return 0;
	}

	Vector3 closestPtWorld = box.GetTransform().Transform(center);

	Contact* contact = data->mContacts;
	contact->mContactNormal = (closestPtWorld - center);
	contact->mContactNormal.Normalize();
	contact->mContactPosition = closestPtWorld;
	contact->mPenetration = sphere.mRadius - precision_sqrt(distance);
	contact->setBodyData(box.mBody, sphere.mBody, data->mFriction, data->mRestitution);

	data->AddContacts(1);
	return 1;
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index) \
    if (!CollisionDetector::TryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;


uint32_t CollisionDetector::BoxAndBox(const CollisionBox &one, const CollisionBox &two, CollisionData *data)
{
	if (!IntersectionTests::BoxAndBox(one, two))
	{
		return 0;
	}

	// Find the vector between the two centres
	Vector3 toCentre = two.GetAxis(3) - one.GetAxis(3);

	// We start assuming there is no contact
	Precision pen = PRECISION_MAX;
	uint32_t best = 0xffffff;

	// Now we check each axes, returning if it gives us
	// a separating axis, and keeping track of the axis with
	// the smallest penetration otherwise.
	CHECK_OVERLAP(one.GetAxis(0), 0);
	CHECK_OVERLAP(one.GetAxis(1), 1);
	CHECK_OVERLAP(one.GetAxis(2), 2);

	CHECK_OVERLAP(two.GetAxis(0), 3);
	CHECK_OVERLAP(two.GetAxis(1), 4);
	CHECK_OVERLAP(two.GetAxis(2), 5);

	// Store the best axis-major, in case we run into almost
	// parallel edge collisions later
	unsigned bestSingleAxis = best;

	CHECK_OVERLAP(one.GetAxis(0).VectorProduct(two.GetAxis(0)), 6);
	CHECK_OVERLAP(one.GetAxis(0).VectorProduct(two.GetAxis(1)), 7);
	CHECK_OVERLAP(one.GetAxis(0).VectorProduct(two.GetAxis(2)), 8);
	CHECK_OVERLAP(one.GetAxis(1).VectorProduct(two.GetAxis(0)), 9);
	CHECK_OVERLAP(one.GetAxis(1).VectorProduct(two.GetAxis(1)), 10);
	CHECK_OVERLAP(one.GetAxis(1).VectorProduct(two.GetAxis(2)), 11);
	CHECK_OVERLAP(one.GetAxis(2).VectorProduct(two.GetAxis(0)), 12);
	CHECK_OVERLAP(one.GetAxis(2).VectorProduct(two.GetAxis(1)), 13);
	CHECK_OVERLAP(one.GetAxis(2).VectorProduct(two.GetAxis(2)), 14);

	// Make sure we've got a result.
	assert(best != 0xffffff);

	// We now know there's a collision, and we know which
	// of the axes gave the smallest penetration. We now
	// can deal with it in different ways depending on
	// the case.
	if (best < 3)
	{
		// We've got a vertex of box two on a face of box one.
		FillPointFaceBoxBox(one, two, toCentre, data, best, pen);
		data->AddContacts(1);
		return 1;
	}
	else if (best < 6)
	{
		// We've got a vertex of box one on a face of box two.
		// We use the same algorithm as above, but swap around
		// one and two (and therefore also the vector between their
		// centres).
		FillPointFaceBoxBox(two, one, toCentre*-1.0f, data, best - 3, pen);
		data->AddContacts(1);
		return 1;
	}
	else
	{
		// We've got an edge-edge contact. Find out which axes
		best -= 6;
		uint32_t oneAxisIndex = best / 3;
		uint32_t twoAxisIndex = best % 3;
		Vector3 oneAxis = one.GetAxis(oneAxisIndex);
		Vector3 twoAxis = two.GetAxis(twoAxisIndex);
		Vector3 axis = oneAxis.VectorProduct(twoAxis);
		axis.Normalize();

		// The axis should point from box one to box two.
		if (axis.ScalarProduct(toCentre) > 0)
		{
			axis = axis * -1.0f;
		}

		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.
		Vector3 ptOnOneEdge = one.halfSize;
		Vector3 ptOnTwoEdge = two.halfSize;
		for (unsigned i = 0; i < 3; i++)
		{
			if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
			else if (one.GetAxis(i).ScalarProduct(axis) > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

			if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
			else if (two.GetAxis(i).ScalarProduct(axis)  < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
		}

		// Move them into world coordinates (they are already oriented
		// correctly, since they have been derived from the axes).
		ptOnOneEdge = one.GetTransform() * ptOnOneEdge;
		ptOnTwoEdge = two.GetTransform() * ptOnTwoEdge;

		// So we have a point and a direction for the colliding edges.
		// We need to find out point of closest approach of the two
		// line-segments.
		Vector3 vertex = ContactPoint(
			ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
			ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
			bestSingleAxis > 2
		);

		// We can fill the contact.
		Contact* contact = data->mContacts;

		contact->mPenetration = pen;
		contact->mContactNormal = axis;
		contact->mContactPosition = vertex;
		contact->setBodyData(one.mBody, two.mBody, data->mFriction, data->mRestitution);
		data->AddContacts(1);
		return 1;
	}
}
#undef CHECK_OVERLAP

