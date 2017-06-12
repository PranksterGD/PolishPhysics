#pragma once
#include "Precision.h"
#include "Vector3.h"

namespace PolishPhysics
{
	class Quaternion
	{
	public:

		union
		{
			struct 
			{
				/**Holds the real component of the quaternion */
				Precision r;

				/**Holds the first complex component of the quaternion */
				Precision i;

				/**Holds the second complex component of the quaternion */
				Precision j;

				/**Holds the third complex component of the quaternion */
				Precision k;
			};

			/**Holds the quaternion data in array form.*/
			Precision mData[4];
		};

		Quaternion(Precision w = 0, Precision x = 0, Precision y = 0, Precision z = 0);

		void Normalize();

		/**Multiples the quaternion by the other one.
		* @param other- The other quaternion to be multiplied by.*/
		void operator*=(const Quaternion& other);

		/**Rotates the quaternion by the vector.
		* @param vector - The vector to rotate by.*/
		void RotateByVector(const Vector3& vector);

		/**Adds the given vector to this one, scaled by the given amount.
		This is used to update the orientation quaternion by a rotation and time.
		* @param vector- The vector to add.
		* @param scale - The amount of the vector to add.*/
		void AddScaledVector(const Vector3& vector, Precision scale);
	};

}