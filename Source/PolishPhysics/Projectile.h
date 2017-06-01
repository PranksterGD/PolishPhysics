#pragma once
#include "Particle.h"
#include <cstdint>

namespace PolishPhysics
{
	class Projectile
	{
	public:

		enum class ShotType
		{
			PISTOL,
			CROSSBOW,
			INVALID,
		};

		Projectile();

		void SetType(ShotType type);

		ShotType GetType() const;

		Vector3 GetPosition() const;

		void Fire();

		bool IsExpired() const;

		virtual void Update();

		virtual ~Projectile() = default;

	private:

		Particle mParticle;

		ShotType mType;

		bool mExpired;
	};
}
