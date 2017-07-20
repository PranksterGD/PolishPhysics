#include "pch.h"

using namespace std;
using namespace PolishPhysics;

RigidBody::RigidBody(Precision gravity) :
	mGravity(gravity), mLinearDamping(0.99f), mAngularDamping(0.99f)
{

}


void RigidBody::SetIntertiaTensor(const Matrix3& intertiaTensor)
{
	mInverseInertiaTensor.SetInverse(intertiaTensor);
}

void RigidBody::AddForce(const Vector3& force)
{
	mAccumulatedForce += force;
}

void RigidBody::AddForceAtPoint(const Vector3& force, const Vector3& point)
{
	Vector3 relativePos = point;
	relativePos -= mPosition;

	mAccumulatedForce += force;
	mAccumulatedTorque += relativePos.VectorProduct(force);
}

void RigidBody::AddForceAtBodyPoint(const Vector3& force, const Vector3& point)
{
	Vector3 pt = GetPointInWorldSpace(point);

	AddForceAtPoint(force, point);
}

Vector3 RigidBody::GetPointInLocalSpace(const Vector3 &point) const
{
	return mTransformMatrix.TransformInverse(point);
}

Vector3 RigidBody::GetPointInWorldSpace(const Vector3 &point) const
{
	return mTransformMatrix.Transform(point);
}


bool RigidBody::HasInfiniteMass() const
{
	return mInverseMass == 0;
}

void RigidBody::SetMass(Precision mass)
{
	if (mass != 0.0f)
	{
		mInverseMass = 1 / mass;
	}
	else
	{
		mInverseMass = 0.0f;
	}
}

void RigidBody::SetInverseMass(Precision inverseMass)
{
	mInverseMass = inverseMass;
}

Precision RigidBody::GetMass() const
{
	return 1 / mInverseMass;
}

Precision RigidBody::GetInverseMass() const
{
	return mInverseMass;
}

Precision RigidBody::GetLinearDamping() const
{
	return mLinearDamping;
}

void RigidBody::SetLinearDamping(Precision damping)
{
	mLinearDamping = damping;
}

Precision RigidBody::GetAngularDamping() const
{
	return mAngularDamping;
}

void RigidBody::SetAngularDamping(Precision damping)
{
	mAngularDamping = damping;
}

Vector3 RigidBody::GetPosition() const
{
	return mPosition;
}

void RigidBody::SetPosition(const Vector3& position)
{
	mPosition = position;
}

Quaternion RigidBody::GetOrientation() const
{
	return mOrientation;
}

void RigidBody::SetOrientation(const Quaternion& orientation)
{
	mOrientation = orientation;
}

Vector3 RigidBody::GetVelocity() const
{
	return mVelocity;
}

void RigidBody::SetVelocity(const Vector3& velocity)
{
	mVelocity = velocity;
}

Vector3 RigidBody::GetRotation() const
{
	return mRotation;
}

void RigidBody::SetRotation(const Vector3& rotation)
{
	mRotation = rotation;
}

Vector3 RigidBody::GetAcceleration() const
{
	return mAcceleration;
}

void RigidBody::SetAcceleration(const Vector3& acceleration)
{
	mAcceleration = acceleration;
}

void RigidBody::ClearAccumulators()
{
	mAccumulatedForce.Clear();
	mAccumulatedTorque.Clear();
}

void RigidBody::CalulateTransformMatrix(Matrix4& transformMatrix, const Vector3& position, const Quaternion& orientation)
{
	transformMatrix.mData[0] = 1 - 2 * orientation.j*orientation.j - 2 * orientation.k*orientation.k;
	transformMatrix.mData[1] = 2 * orientation.i*orientation.j - 2 * orientation.r*orientation.k;
	transformMatrix.mData[2] = 2 * orientation.i*orientation.k + 2 * orientation.r*orientation.j;
	transformMatrix.mData[3] = position.X;

	transformMatrix.mData[4] = 2 * orientation.i*orientation.j + 2 * orientation.r*orientation.k;
	transformMatrix.mData[5] = 1 - 2 * orientation.i*orientation.i -2 * orientation.k*orientation.k;
	transformMatrix.mData[6] = 2 * orientation.j*orientation.k - 2 * orientation.r*orientation.i;
	transformMatrix.mData[7] = position.Y;

	transformMatrix.mData[8] = 2 * orientation.i*orientation.k - 2 * orientation.r*orientation.j;
	transformMatrix.mData[9] = 2 * orientation.j*orientation.k + 2 * orientation.r*orientation.i;
	transformMatrix.mData[10] = 1 - 2 * orientation.i*orientation.i - 2 * orientation.j*orientation.j;
	transformMatrix.mData[11] = position.Z;
}

void RigidBody::TransformIntertiaTensor(Matrix3& intertiaWorld, const Quaternion& orientation, const Matrix3& intertiaBody, const Matrix4& rotationMatrix)
{
	UNREFERENCED_PARAMETER(orientation);

	Precision t4 = rotationMatrix.mData[0] * intertiaBody.mData[0] + rotationMatrix.mData[1] * intertiaBody.mData[3] +
		rotationMatrix.mData[2] * intertiaBody.mData[6];
	Precision t9 = rotationMatrix.mData[0] * intertiaBody.mData[1] +
		rotationMatrix.mData[1] * intertiaBody.mData[4] +
		rotationMatrix.mData[2] * intertiaBody.mData[7];
	Precision t14 = rotationMatrix.mData[0] * intertiaBody.mData[2] +
		rotationMatrix.mData[1] * intertiaBody.mData[5] +
		rotationMatrix.mData[2] * intertiaBody.mData[8];
	Precision t28 = rotationMatrix.mData[4] * intertiaBody.mData[0] +
		rotationMatrix.mData[5] * intertiaBody.mData[3] +
		rotationMatrix.mData[6] * intertiaBody.mData[6];
	Precision t33 = rotationMatrix.mData[4] * intertiaBody.mData[1] +
		rotationMatrix.mData[5] * intertiaBody.mData[4] +
		rotationMatrix.mData[6] * intertiaBody.mData[7];
	Precision t38 = rotationMatrix.mData[4] * intertiaBody.mData[2] +
		rotationMatrix.mData[5] * intertiaBody.mData[5] +
		rotationMatrix.mData[6] * intertiaBody.mData[8];
	Precision t52 = rotationMatrix.mData[8] * intertiaBody.mData[0] +
		rotationMatrix.mData[9] * intertiaBody.mData[3] +
		rotationMatrix.mData[10] * intertiaBody.mData[6];
	Precision t57 = rotationMatrix.mData[8] * intertiaBody.mData[1] +
		rotationMatrix.mData[9] * intertiaBody.mData[4] +
		rotationMatrix.mData[10] * intertiaBody.mData[7];
	Precision t62 = rotationMatrix.mData[8] * intertiaBody.mData[2] +
		rotationMatrix.mData[9] * intertiaBody.mData[5] +
		rotationMatrix.mData[10] * intertiaBody.mData[8];

	intertiaWorld.mData[0] = t4*rotationMatrix.mData[0] +
		t9*rotationMatrix.mData[1] +
		t14*rotationMatrix.mData[2];
	intertiaWorld.mData[1] = t4*rotationMatrix.mData[4] +
		t9*rotationMatrix.mData[5] +
		t14*rotationMatrix.mData[6];
	intertiaWorld.mData[2] = t4*rotationMatrix.mData[8] +
		t9*rotationMatrix.mData[9] +
		t14*rotationMatrix.mData[10];
	intertiaWorld.mData[3] = t28*rotationMatrix.mData[0] +
		t33*rotationMatrix.mData[1] +
		t38*rotationMatrix.mData[2];
	intertiaWorld.mData[4] = t28*rotationMatrix.mData[4] +
		t33*rotationMatrix.mData[5] +
		t38*rotationMatrix.mData[6];
	intertiaWorld.mData[5] = t28*rotationMatrix.mData[8] +
		t33*rotationMatrix.mData[9] +
		t38*rotationMatrix.mData[10];
	intertiaWorld.mData[6] = t52*rotationMatrix.mData[0] +
		t57*rotationMatrix.mData[1] +
		t62*rotationMatrix.mData[2];
	intertiaWorld.mData[7] = t52*rotationMatrix.mData[4] +
		t57*rotationMatrix.mData[5] +
		t62*rotationMatrix.mData[6];
	intertiaWorld.mData[8] = t52*rotationMatrix.mData[8] +
		t57*rotationMatrix.mData[9] +
		t62*rotationMatrix.mData[10];
}

void RigidBody::Integrate(Precision deltaTime)
{
	Vector3 lastFrameAcc = mAcceleration;
	lastFrameAcc.AddScaledVector(mAccumulatedForce, mInverseMass);
	lastFrameAcc += Vector3(0, mGravity, 0);

	Vector3 angularAcc = mInverseInertiaTensorWorld.Transform(mAccumulatedTorque);

	mVelocity.AddScaledVector(lastFrameAcc, deltaTime);
	mRotation.AddScaledVector(angularAcc, deltaTime);

	mVelocity *= precision_pow(mLinearDamping, deltaTime);
	mRotation *= precision_pow(mAngularDamping, deltaTime);

	mPosition.AddScaledVector(mVelocity, deltaTime);
	mOrientation.AddScaledVector(mRotation, deltaTime);

	CalculateDerivedData();

	ClearAccumulators();
}

void RigidBody::CalculateDerivedData()
{
	mOrientation.Normalize();

	CalulateTransformMatrix(mTransformMatrix, mPosition, mOrientation);

	TransformIntertiaTensor(mInverseInertiaTensorWorld, mOrientation, mInverseInertiaTensor, mTransformMatrix);
}

Matrix4 RigidBody::GetTransform() const
{
	return mTransformMatrix;
}

void RigidBody::GetLastFrameAcceleration(Vector3 *linearAcceleration) const
{
	*linearAcceleration = mLastFrameAcceleration;
}

Vector3 RigidBody::GetLastFrameAcceleration() const
{
	return mLastFrameAcceleration;
}

bool RigidBody::GetAwake() const
{
	return mIsAwake;
}

void RigidBody::SetAwake(bool wake)
{
	mIsAwake = wake;
}

Matrix3 RigidBody::GetInverseInertiaTensorWorld() const
{
	return mInverseInertiaTensorWorld;
}

void RigidBody::AddVelocity(const Vector3& velocity)
{
	mVelocity += velocity;
}

void RigidBody::AddRotation(const Vector3& rotation)
{
	mRotation += rotation;
}

void RigidBody::GetGLTransform(float matrix[16])
{
	matrix[0] = (float)mTransformMatrix.mData[0];
	matrix[1] = (float)mTransformMatrix.mData[4];
	matrix[2] = (float)mTransformMatrix.mData[8];
	matrix[3] = 0;

	matrix[4] = (float)mTransformMatrix.mData[1];
	matrix[5] = (float)mTransformMatrix.mData[5];
	matrix[6] = (float)mTransformMatrix.mData[9];
	matrix[7] = 0;

	matrix[8] = (float)mTransformMatrix.mData[2];
	matrix[9] = (float)mTransformMatrix.mData[6];
	matrix[10] = (float)mTransformMatrix.mData[10];
	matrix[11] = 0;

	matrix[12] = (float)mTransformMatrix.mData[3];
	matrix[13] = (float)mTransformMatrix.mData[7];
	matrix[14] = (float)mTransformMatrix.mData[11];
	matrix[15] = 1;
}

void RigidBody::SetInverseIntertiaTensor(const Matrix3& tensor)
{
	mInverseInertiaTensor = tensor;
}