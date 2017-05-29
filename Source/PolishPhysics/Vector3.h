#pragma once
#include "Precision.h"
namespace PolishPhysics
{
	/**Represents a point in 3 dimensional space. */
	class Vector3
	{
	public:

		/**Value along the x axis. */
		precision X;

		/**Value along the y axis. */
		precision Y;

		/**Value along the z- axis. */
		precision Z;

	private:

		/**Padding to ensure 4 word alignment. */
		precision mPad;

	public:
		/**Default constructor for the vector that assigns X, Y and Z to zero. */
		Vector3();

		/**Constructor for the vector.
		*@param x- The X Value.
		*@param y- The Y Value. 
		*@param z- The Z Value. */
		Vector3(precision x, precision y, precision z);

		/**Copy constructor for the vector.
		* @param other- The other vector to be copied from. */
		Vector3(const Vector3& other);

		/**Move constructor for the vector.
		* @param other - The other vector to be moved to. */
		Vector3(Vector3&& other);

		/**Assignment operator for the vector.
		* @param other - The other vector to be copied from. */
		Vector3& operator=(const Vector3& other);

		/**Move assignment operator for the vector.
		* @param other - The other vector to be moved to. */
		Vector3& operator=(Vector3&& other);

		/**Equality operator for the vector.
		* @param other - The other vector to be compared against.
		* @return A boolean - True if the vectors are equal, false otherwise.*/
		bool operator==(const Vector3& other) const ;

		/**Inequality operator for the vector.
		* @param other - The other vector to be compared against.
		* @return A boolean - True if the vectors are not equal, false otherwise.*/
		bool operator!=(const Vector3& other) const;

		/**Function that negates all the components of the vector. */
		void Invert();

		/**Function that returns the magnitude of the vector.
		* @return - A precision that is the magnitude of the vector*/
		precision Magnitude() const;

		/**Function that returns the squared magnitude of the vector.
		* @return - A precision that is the squared magnitude of the vector*/
		precision SquareMagnitude() const;

		void Normalize();

		/**Multiplies the vector by the given scalar.
		* @param scalar - The scalar to be multiplied by.*/
		void operator*=(precision scalar);

		/**Returns a copy of the vector, multiplied by the given scalar.
		* @param scalar - The scalar to be multiplied by.
		* @return - A copy of the vector after multiplication.*/
		Vector3 operator*(precision scalar) const;

		/**Adds the vector by another vector.
		* @param other - The vector to be added by.*/
		void operator+=(const Vector3& other);

		/**Returns a copy of the vector, added with another vector.
		* @return - A copy of the vector after addition.
		* @param other - The scalar to be multiplied by.*/
		Vector3 operator+(const Vector3& other) const;

		/**Subtracts the vector by another vector.
		* @param other - The vector to be added by.*/
		void operator-=(const Vector3& other);

		/**Returns a copy of the vector, subtracted by another vector.
		* @param other - The vector to be subtracted by.
		* @return - A copy of the vector after subtraction.*/
		Vector3 operator-(const Vector3& other) const;

		/**Adds the vector by another vector, multiplied by a scale.
		* @param other - The vector to be added by.
		* @param scale - The scale by which the other vector should be multipled by.*/
		void AddScaledVector(const Vector3& other, precision scale);

		/**Function that multiples each component of the vector by the corresponding
		components of the other vector.
		* @param other - The vector to be subtracted by.*/
		void ComponentProductAssignment(const Vector3& other);

		/**Function that returns a new vector that is the component-wise product of the 
		vector and the other vector.
		* @param other - The vector to be subtracted by.
		* @return - A copy of the vector after multiplication.*/
		Vector3 ComponentProduct(const Vector3& other) const;

		/**Function that returns the scalar product of the vector with the other vector.
		* @param other - The other vector.
		* @return - A precision that represents the dot product.*/
		precision ScalarProduct(const Vector3& other) const;

		/**Function that returns the vector product of the vector with the other vector.
		* @param other - The other vector.
		* @return - A vector that represents the vector product.*/
		Vector3 VectorProduct(const Vector3& other) const;

		/**Function that sets the vector to be the result of the vector product with
		the other vector.
		* @param other - The vector to be subtracted by.*/
		void VectorProductAssignment(const Vector3& other);
	};
}