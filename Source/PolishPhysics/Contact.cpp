#include "pch.h"

using namespace std;
using namespace PolishPhysics;

void Contact::setBodyData(RigidBody* one, RigidBody *two, Precision friction, Precision restitution)
{
	mBodies[0] = one;
	mBodies[1] = two;
	mFriction = friction;
	mRestitution = restitution;
}

void Contact::CalculateContactBasis()
{
	Vector3 contactTangent[2];

	//Check if the Z axis is closer to the X or Y axis
	if (precision_abs(mContactNormal.X) > precision_abs(mContactNormal.Y))
	{
		//Scaling factor to ensure the results are normalized.
		const Precision scale = (Precision) 1.0f / precision_sqrt(mContactNormal.Z * mContactNormal.X +
			mContactNormal.X * mContactNormal.X);

		//The new X asis is at right angles to the world Y axis;
		contactTangent[0].X = mContactNormal.Z * scale;
		contactTangent[0].Y = 0;
		contactTangent[0].Z = -mContactNormal.X *scale;

		//The new Y asis is at right angles to the new X and Z axis;
		contactTangent[1].X = mContactNormal.Y * contactTangent[0].X;
		contactTangent[1].Y = mContactNormal.Z * contactTangent[0].X - mContactNormal.X * contactTangent[0].Z;
		contactTangent[1].Z = -mContactNormal.Y * contactTangent[0].X;
	}
	else
	{
		// Scaling factor to ensure the results are normalised
		const Precision scale = (Precision)1.0 / precision_sqrt(mContactNormal.Z * mContactNormal.Z +
			mContactNormal.Y * mContactNormal.Y);

		// The new X-axis is at right angles to the world X-axis
		contactTangent[0].X = 0;
		contactTangent[0].Y = -mContactNormal.Z * scale;
		contactTangent[0].Z = mContactNormal.Y * scale;

		// The new Y-axis is at right angles to the new X- and Z- axes
		contactTangent[1].X = mContactNormal.Y*contactTangent[0].Z -
			mContactNormal.Z*contactTangent[0].Y;
		contactTangent[1].Y = -mContactNormal.X * contactTangent[0].Z;
		contactTangent[1].Z = mContactNormal.X * contactTangent[0].Y;
	}
}

Vector3 Contact::CalculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor)
{
	Vector3 impulseContact;

	// Build a vector that shows the change in velocity in
	// world space for a unit impulse in the direction of the contact
	// normal.
	Vector3 deltaVelWorld = mRelativeContactPosition[0].VectorProduct(mContactNormal);
	deltaVelWorld = inverseInertiaTensor[0].Transform(deltaVelWorld);
	deltaVelWorld = deltaVelWorld.VectorProduct(mRelativeContactPosition[0]);

	// Work out the change in velocity in contact coordiantes.
	Precision deltaVelocity = deltaVelWorld.ScalarProduct(mContactNormal);

	// Add the linear component of velocity change
	deltaVelocity += mBodies[0]->GetInverseMass();

	// Check if we need to the second body's data
	if (mBodies[1])
	{
		// Go through the same transformation sequence again
		deltaVelWorld = mRelativeContactPosition[1].VectorProduct(mContactNormal);
		deltaVelWorld = inverseInertiaTensor[1].Transform(deltaVelWorld);
		deltaVelWorld = deltaVelWorld.VectorProduct(mRelativeContactPosition[1]);

		// Add the change in velocity due to rotation
		deltaVelocity += deltaVelWorld.ScalarProduct(mContactNormal);

		// Add the change in velocity due to linear motion
		deltaVelocity += mBodies[1]->GetInverseMass();
	}

	// Calculate the required size of the impulse
	impulseContact.X = mDesiredDeltaVelocity / deltaVelocity;
	impulseContact.Y = 0;
	impulseContact.Z = 0;
	return impulseContact;
}

void Contact::SwapBodies()
{
	mContactNormal *= -1;

	RigidBody* temp = mBodies[0];
	mBodies[0] = mBodies[1];
	mBodies[1] = temp;
}

void Contact::CaluclateInternals(Precision deltaTime)
{
	if (mBodies[0] == nullptr)
	{
		SwapBodies();
	}

	assert(mBodies[0] != nullptr);

	//Calculate the contact space transform matrix
	CalculateContactBasis();

	//Calculate the relative positions
	mRelativeContactPosition[0] = mContactPosition - mBodies[0]->GetPosition();

	if (mBodies[1] != nullptr)
	{
		mRelativeContactPosition[1] = mContactPosition - mBodies[1]->GetPosition();
	}

	//Calculate the closing velocity
	mContactVelocity = CalculateLocalVelocity(0, deltaTime);

	if (mBodies[1] != nullptr)
	{
		mContactVelocity -= CalculateLocalVelocity(1, deltaTime);
	}

	CalculateDeltaVelocity(deltaTime);
}

Vector3 Contact::CalculateLocalVelocity(std::uint32_t bodyIndex, Precision deltaTime)
{
	RigidBody* body = mBodies[bodyIndex];

	//Calculate the total velocity of the body.
	Vector3 velocity = body->GetRotation().VectorProduct(mRelativeContactPosition[bodyIndex]);
	velocity += body->GetVelocity();

	//Convert velocity to contact space.
	Vector3 contactVelocity = mContactToWorld.TransformTranspose(velocity);

	//Calculate the amount of velocity that is due to to forces without reactions.

	// Calculate the ammount of velocity that is due to forces without
	// reactions.
	Vector3 accVelocity = body->GetLastFrameAcceleration() * deltaTime;

	// Calculate the velocity in contact-coordinates.
	accVelocity = mContactToWorld.TransformTranspose(accVelocity);

	// We ignore any component of acceleration in the contact normal
	// direction, we are only interested in planar acceleration
	accVelocity.X = 0;

	contactVelocity += accVelocity;

	return contactVelocity;
}

void Contact::MatchAwakeState()
{
	// Collisions with the world never cause a body to wake up.
	if (!mBodies[1])
	{
		return;
	}

	bool body0awake = mBodies[0]->GetAwake();
	bool body1awake = mBodies[1]->GetAwake();

	// Wake up only the sleeping one
	if (body0awake ^ body1awake)
	{
		if (body0awake)
		{
			mBodies[1]->SetAwake(true);
		}
		else
		{
			mBodies[0]->SetAwake(true);
		}
	}
}

void Contact::ApplyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], Precision penetration)
{
	const Precision angularLimit = (Precision)0.2f;
	Precision angularMove[2];
	Precision linearMove[2];

	Precision totalInertia = 0;
	Precision linearInertia[2];
	Precision angularInertia[2];

	// We need to work out the inertia of each object in the direction
	// of the contact normal, due to angular inertia only.
	for (unsigned i = 0; i < 2; i++)
	{
		if (mBodies[i] != nullptr)
		{
			Matrix3 inverseInertiaTensor = mBodies[i]->GetInverseInertiaTensorWorld();

			// Use the same procedure as for calculating frictionless
			// velocity change to work out the angular inertia.
			Vector3 angularInertiaWorld = mRelativeContactPosition[i].VectorProduct(mContactNormal);
			angularInertiaWorld = inverseInertiaTensor.Transform(angularInertiaWorld);
			angularInertiaWorld = angularInertiaWorld.VectorProduct(mRelativeContactPosition[i]);
			angularInertia[i] = angularInertiaWorld.ScalarProduct(mContactNormal);

			// The linear component is simply the inverse mass
			linearInertia[i] = mBodies[i]->GetInverseMass();

			// Keep track of the total inertia from all components
			totalInertia += linearInertia[i] + angularInertia[i];

			// We break the loop here so that the totalInertia value is
			// completely calculated (by both iterations) before
			// continuing.
		}
	}

	// Loop through again calculating and applying the changes
	for (unsigned i = 0; i < 2; i++)
	{
		if (mBodies[i])
		{
			// The linear and angular movements required are in proportion to
			// the two inverse inertias.
			Precision sign = (Precision) (i == 0) ? 1.0f : -1.0f;

			angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
			linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

			// To avoid angular projections that are too great (when mass is large
			// but inertia tensor is small) limit the angular move.
			Vector3 projection = mRelativeContactPosition[i];
			projection.AddScaledVector(mContactNormal, -mRelativeContactPosition[i].ScalarProduct(mContactNormal));

			// Use the small angle approximation for the sine of the angle (i.e.
			// the magnitude would be sine(angularLimit) * projection.magnitude
			// but we approximate sine(angularLimit) to angularLimit).
			Precision maxMagnitude = angularLimit * projection.Magnitude();

			if (angularMove[i] < -maxMagnitude)
			{
				Precision totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = -maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}
			else if (angularMove[i] > maxMagnitude)
			{
				Precision totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}

			// We have the linear amount of movement required by turning
			// the rigid body (in angularMove[i]). We now need to
			// calculate the desired rotation to achieve that.
			if (angularMove[i] == 0)
			{
				// Easy case - no angular movement means no rotation.
				angularChange[i].Clear();
			}
			else
			{
				// Work out the direction we'd like to rotate in.
				Vector3 targetAngularDirection = mRelativeContactPosition[i].VectorProduct(mContactNormal);

				Matrix3 inverseInertiaTensor = mBodies[i]->GetInverseInertiaTensorWorld();

				// Work out the direction we'd need to rotate to achieve that
				angularChange[i] = inverseInertiaTensor.Transform(targetAngularDirection) *
					(angularMove[i] / angularInertia[i]);
			}

			// Velocity change is easier - it is just the linear movement
			// along the contact normal.
			linearChange[i] = mContactNormal * linearMove[i];

			// Now we can start to apply the values we've calculated.
			// Apply the linear movement
			Vector3 pos = mBodies[i]->GetPosition();
			pos.AddScaledVector(mContactNormal, linearMove[i]);
			mBodies[i]->SetPosition(pos);

			// And the change in orientation
			Quaternion q = mBodies[i]->GetOrientation();
			q.AddScaledVector(angularChange[i], ((Precision)1.0));
			mBodies[i]->SetOrientation(q);

			// We need to calculate the derived data for any body that is
			// asleep, so that the changes are reflected in the object's
			// data. Otherwise the resolution will not change the position
			// of the object, and the next collision detection round will
			// have the same penetration.
			if (!mBodies[i]->GetAwake())
			{
				mBodies[i]->CalculateDerivedData();
			}
		}
	}
}

void Contact::ApplyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2])
{
	// Get hold of the inverse mass and inverse inertia tensor, both in
	// world coordinates.
	Matrix3 inverseInertiaTensor[2];
	inverseInertiaTensor[1] = mBodies[0]->GetInverseInertiaTensorWorld();

	if (mBodies[1] != nullptr)
	{
		inverseInertiaTensor[1] = mBodies[1]->GetInverseInertiaTensorWorld();
	}

	// We will calculate the impulse for each contact axis
	Vector3 impulseContact;

	if (mFriction == (Precision)0.0)
	{
		// Use the short format for frictionless contacts
		impulseContact = CalculateFrictionlessImpulse(inverseInertiaTensor);
	}
	else
	{
		// Otherwise we may have impulses that aren't in the direction of the
		// contact, so we need the more complex version.
	//	impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
	}

	// Convert impulse to world coordinates
	Vector3 impulse = mContactToWorld.Transform(impulseContact);

	// Split in the impulse into linear and rotational components
	Vector3 impulsiveTorque = mRelativeContactPosition[0].VectorProduct(impulse);
	rotationChange[0] = inverseInertiaTensor[0].Transform(impulsiveTorque);
	velocityChange[0].Clear();
	velocityChange[0].AddScaledVector(impulse, mBodies[0]->GetInverseMass());

	// Apply the changes
	mBodies[0]->AddVelocity(velocityChange[0]);
	mBodies[0]->AddRotation(rotationChange[0]);

	if (mBodies[1] != nullptr)
	{
		// Work out body one's linear and angular changes
		impulsiveTorque = impulse.VectorProduct(mRelativeContactPosition[1]);
		rotationChange[1] = inverseInertiaTensor[1].Transform(impulsiveTorque);
		velocityChange[1].Clear();
		velocityChange[1].AddScaledVector(impulse, -mBodies[1]->GetInverseMass());
	}

	mBodies[1]->AddVelocity(velocityChange[1]);
	mBodies[1]->AddRotation(rotationChange[1]);
}

void Contact::CalculateDeltaVelocity(Precision deltaTime)
{
	const static Precision velocityLimit = (Precision)0.25f;

	// Calculate the acceleration induced velocity accumulated this frame
	Precision velocityFromAcc = 0;

	if (mBodies[0]->GetAwake())
	{
		velocityFromAcc += (mBodies[0]->GetLastFrameAcceleration() * deltaTime).ScalarProduct(mContactNormal);
	}

	if (mBodies[1] != nullptr && mBodies[1]->GetAwake())
	{
		velocityFromAcc += (mBodies[1]->GetLastFrameAcceleration() * deltaTime).ScalarProduct(mContactNormal);

	}

	// If the velocity is very slow, limit the restitution
	Precision restitution = mRestitution;
	if (precision_abs(mContactVelocity.X) < velocityLimit)
	{
		restitution = (Precision)0.0f;
	}

	// Combine the bounce velocity with the removed
	// acceleration velocity.
	mDesiredDeltaVelocity = -mContactVelocity.X - restitution * (mContactVelocity.X - velocityFromAcc);
}
