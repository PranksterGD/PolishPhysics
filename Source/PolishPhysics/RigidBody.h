#pragma once
#include "Vector3.h"
#include "Matrix4.h"
#include "Matrix3.h"
#include "Quaternion.h"

namespace PolishPhysics
{
	/**A rigid body is the basic simulation object in the physics engine. */
	class RigidBody
	{
		friend class RigidBodyWorld;

	public:

		RigidBody(Precision gravity = 10.0f);

		/**Sets the inverse inertia tensor to be the inverse of the matrix passed in.
		* @param intertiaTensor - the inverse of the new inertia tensor.*/
		void SetIntertiaTensor(const Matrix3& intertiaTensor);

		/**Adds a force to the center of mass of the rigid body. 
		* @param force- The force to add.*/
		void AddForce(const Vector3& force);

		/**Adds a force to the given point on the rigid body, specified in world space.
		* @param force- The force to add.
		* @param point - The point on the body in world space.*/
		void AddForceAtPoint(const Vector3& force, const Vector3& point);

		/**Adds a force to the given point on the rigid body, specified in body space.
		* @param force- The force to add.
		* @param point - The point on the body in body space.*/
		void AddForceAtBodyPoint(const Vector3& force, const Vector3& point);

		/**Converts a point in world space to body space.
		* @param point - The point in world space.
		* @return - A vector that represents the point in body space.*/
		Vector3 GetPointInLocalSpace(const Vector3 &point) const;

		/**Converts a point in body space to world space.
		* @param point - The point in body space.
		* @return - A vector that represents the point in world space.*/
		Vector3 GetPointInWorldSpace(const Vector3 &point) const;

		bool HasInfiniteMass() const;

		void SetMass(Precision mass);

		void SetInverseMass(Precision inverseMass);

		Precision GetMass() const;

		Precision GetInverseMass() const;

		Precision GetLinearDamping() const;

		void SetLinearDamping(Precision damping);

		Precision GetAngularDamping() const;

		void SetAngularDamping(Precision damping);

		Vector3 GetPosition() const;

		void SetPosition(const Vector3& position);

		Quaternion GetOrientation() const;

		void SetOrientation(const Quaternion& orientation);

		Vector3 GetVelocity() const;

		void SetVelocity(const Vector3& velocity);

		Vector3 GetRotation() const;

		void SetRotation(const Vector3& rotation);

		Vector3 GetAcceleration() const;

		void SetAcceleration(const Vector3& acceleration);

		Matrix4 GetTransform() const;


	protected:

		/**Holds the  inverse mass of the rigid body. */
		Precision mInverseMass;

		/**Holds the amount of damping applied to linear motion. */
		Precision mLinearDamping;

		/**Holds the amount of damping applied to angular motion. */
		Precision mAngularDamping;

		/**Holds the linear position of the rigid body in world space. */
		Vector3 mPosition;

		/**Holds the angular orientation of the rigid body in world space. */
		Quaternion mOrientation;

		/**Holds the linear velocity of the rigid body in world space. */
		Vector3 mVelocity;

		/**Holds the angular velocity of the rigid body in world space. */
		Vector3 mRotation;

		/**Holds the linear acceleration of the rigid body in world space. */
		Vector3 mAcceleration;

		Precision mGravity;

		/**Holds a transform matrix for converting body space into world space.. */
		Matrix4 mTransformMatrix;

		/**Holds the inverse of the inertia tensor in body space. */
		Matrix3 mInverseInertiaTensor;

		/**Holds the inverse of the inertia tensor in world space. */
		Matrix3 mInverseInertiaTensorWorld;

		Vector3 mAccumulatedForce;
		Vector3 mAccumulatedTorque;

		void ClearAccumulators();

		void Integrate(Precision deltaTime);

		/**Calculates internal data from state data. */
		void CalculateDerviedData();

		void CalulateTransformMatrix(Matrix4& transformMatrix, const Vector3& position, const Quaternion& orientation);

		void TransformIntertiaTensor(Matrix3& intertiaWorld, const Quaternion& orientation, const Matrix3& intertiaBody, const Matrix4& rotationMatrix);
	};
}