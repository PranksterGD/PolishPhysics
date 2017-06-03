#pragma once
#include "ParticleLink.h"

namespace PolishPhysics
{
	class ParticleRod : public ParticleLink
	{
	public:

		/**Holds the length of the rod. */
		Precision mLength;

		virtual std::uint32_t AddContact(class ParticleContact *contact, std::uint32_t limit) const override;
	};
}