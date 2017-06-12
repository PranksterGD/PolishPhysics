#pragma once
#include "Precision.h"
#include "Vector3.h"
#include "Quaternion.h"

namespace PolishPhysics
{
	/**Holds a 3x3 row major matrix representing a transformation in 3D space that does not
	include a translational component.*/
	class Matrix3
	{
	public:

		Matrix3(Precision a = 0, Precision b = 0, Precision c = 0, Precision d = 0, Precision e = 0,
			Precision f = 0, Precision g = 0, Precision h = 0, Precision i = 0);

		/**Holds the tensor matrix data in array form. */
		Precision mData[9];

		/**Transforms the given vector by this matrix.
		* @param vector- The vector to transform.
		* @return - A vector that is the given vector after transformation.*/
		Vector3 operator*(const Vector3& vector) const;
		
		/**Transforms the given vector by this matrix.
		* @param vector - The vector to transform.
		* @return - A vector that is the given vector after transformation.*/
		Vector3 Transform(const Vector3& vector)const;

		/**Returns a matrix that is result of the multiplication of the matrix with the matrix passed in.
		* @param other- The other matrix in the multiplication
		* @return - The matrix that is the result of the multiplication.*/
		Matrix3 operator*(const Matrix3& other) const;

		/**Multiples this matrix in place by the other matrix.
		* @param other- The other matrix to be multiplied by.*/
		void operator*=(const Matrix3& other);

		/**Sets the matrix to be the inverse of the matrix.
		* @param other- The matrix whose inverse is to be taken.*/
		void SetInverse(const Matrix3& other);

		/**Returns a new matrix that is the inverse of the matrix.
		* @return - A new matrix that is the inverse of the matrix. */
		Matrix3 Inverse()const;

		/**Inverses the matrix. */
		void Invert();

		void SetTranspose(const Matrix3& other);

		Matrix3 Transpose() const;

		/**Sets this matrix to be the rotation matrix corresponding to the given quaternion.
		* @Param quaternion - The quaternion whose rotation is to be used.*/
		void SetOrientation(const Quaternion& quaternion);
	};
}