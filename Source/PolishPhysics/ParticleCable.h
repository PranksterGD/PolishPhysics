#pragma once
#include "ParticleLink.h"

namespace PolishPhysics
{
	class ParticleCable : public ParticleLink
	{
	public:

		/**Holds the maximum length of the cable. */
		Precision mMaxLength;

		/**Holds the restitution (bounciness) of the cable. */
		Precision mRestitution;

		virtual std::uint32_t AddContact(class ParticleContact *contact, std::uint32_t limit) const override;
	};
}