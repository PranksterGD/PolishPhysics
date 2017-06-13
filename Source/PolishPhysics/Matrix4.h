#pragma once
#pragma once
#include "Precision.h"
#include "Vector3.h"

namespace PolishPhysics
{
	/**Holds a transform matrix, consisting of a rotation matrix and a position.
	The matrix has 12 elements and it is assumed that the remaining four are (0,0,0,1) ,
	producing a homogeneous matrix.*/
	class Matrix4
	{
	public:

		Matrix4(Precision a = 0, Precision b = 0, Precision c = 0, Precision d = 0, Precision e = 0,
			Precision f = 0, Precision g = 0, Precision h = 0, Precision i = 0, Precision j = 0,
			Precision k = 0, Precision l = 0);

		/**Holds the tensor matrix data in array form. */
		Precision mData[12];

		/**Transforms the given vector by this matrix.
		* @param vector- The vector to transform.
		* @return - A vector that is the given vector after transformation.*/
		Vector3 operator* (const Vector3& vector) const;

		/**Transforms the given vector by this matrix.
		* @param vector- The vector to transform.
		* @return - A vector that is the given vector after transformation.*/
		Vector3 Transform(const Vector3& vector) const;

		/**Transforms the given direction vector by the inverse of this matrix.
		* @param vector- The vector to transform.
		* @return - A vector that is the given vector after transformation.*/
		Vector3 TransformInverse(const Vector3& vector) const;

		/**Transforms the given direction vector by this matrix.
		* @param vector- The vector to transform.
		* @return - A vector that is the given vector after transformation.*/
		Vector3 TransformDirection(const Vector3& vector) const;

		/**Transforms the given vector by the inverse of this matrix.
		* @param vector- The vector to transform.
		* @return - A vector that is the given vector after transformation.*/
		Vector3 TransformInverseDirection(const Vector3& vector) const;

		/**Returns a matrix that is result of the multiplication of the matrix with the matrix passed in.
		* @param other- The other matrix in the multiplication
		* @return - The matrix that is the result of the multiplication.*/
		Matrix4 operator*(const Matrix4& other) const;

		/**Multiples this matrix in place by the other matrix.
		* @param other- The other matrix to be multiplied by.*/
		void operator*=(const Matrix4& other);

		/**Returns the determinant of the matrix.
		* @return - A precision that represents the determinant of the matrix. */
		Precision GetDeterminant() const;

		/**Sets the matrix to be the inverse of the matrix.
		* @param other- The matrix whose inverse is to be taken.*/
		void SetInverse(const Matrix4& other);

		/**Returns a new matrix that is the inverse of the matrix.
		* @return - A new matrix that is the inverse of the matrix. */
		Matrix4 Inverse()const;

		/**Inverses the matrix. */
		void Invert();

		/**Returns a Vector that represents the world coordinates of the local vector after
		applying the transform matrix.
		* @param local - The local coordinates of the vector.
		* @param transform- The transform matrix to be applied.
		* @return - The world coordinates vector.*/
		Vector3 LocalToWorld(const Vector3& local, const Matrix4& transform) const;


		/**Returns a Vector that represents the local coordinates of the world vector before
		applying the transform matrix.
		* @param world - The world coordinates of the vector.
		* @param transform- The transform matrix to be applied.
		* @return - The local coordinates vector.*/
		Vector3 WorldToLocal(const Vector3& world, const Matrix4& transform) const;

		/**Returns a Vector that represents the world coordinates of the local direction vector after
		applying the transform matrix.
		* @param local - The local coordinates of the vector.
		* @param transform- The transform matrix to be applied.
		* @return - The world coordinates vector.*/
		Vector3 LocalToWorldDirection(const Vector3& local, const Matrix4& transform) const;


		/**Returns a Vector that represents the local coordinates of the world direction vector before
		applying the transform matrix.
		* @param world - The world coordinates of the vector.
		* @param transform- The transform matrix to be applied.
		* @return - The local coordinates vector.*/
		Vector3 WorldToLocalDirection(const Vector3& world, const Matrix4& transform) const;

		void SetOrientationAndPosition(const class Quaternion& quaternion, const Vector3& position);
	};
}