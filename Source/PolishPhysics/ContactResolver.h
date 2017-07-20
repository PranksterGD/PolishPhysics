#pragma once
#include "Precision.h"
namespace PolishPhysics
{
	class ContactResolver
	{
	public:

		std::uint32_t mVelocityIterationsUsed;

		std::uint32_t mPositionIterationsUsed;

		void ResolveContacts(class Contact* contactArray, unsigned numContacts, Precision deltaTime);

		ContactResolver(std::uint32_t iterations, Precision velocityEpsilon = (Precision)0.01,
			Precision positionEpsilon = (Precision)0.01);

	protected:

		std::uint32_t mPositionIterations;

		std::uint32_t mVelocityIterations;

		Precision mVelocityEpsilon;

		Precision mPositionEpsilon;
		/**
		* Sets up contacts ready for processing. This makes sure their
		* internal data is configured correctly and the correct set of bodies
		* is made alive.
		*/
		void PrepareContacts(class Contact *contactArray, unsigned numContacts, Precision deltaTime);

		/**
		* Resolves the velocity issues with the given array of constraints,
		* using the given number of iterations.
		*/
		void AdjustVelocities(class Contact *contactArray, unsigned numContacts, Precision deltaTime);

		/**
		* Resolves the positional issues with the given array of constraints,
		* using the given number of iterations.
		*/
		void AdjustPositions(class Contact *contacts, unsigned numContacts, Precision deltaTime);
	};
}