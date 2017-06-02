#pragma once
#include "ParticleForceGenerator.h"
#include "Vector3.h"
namespace PolishPhysics
{
	class ParticleGravityForceGenerator : public ParticleForceGenerator
	{
	private:
		
		/**Holds the acceleration due to gravity. */
		Vector3 mGravity;

	public:

		ParticleGravityForceGenerator(const Vector3& gravity);

		virtual void UpdateForce(class Particle& particle, Precision deltaTime);
	};
}