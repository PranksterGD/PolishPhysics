#pragma once
#include <cstdint>
namespace PolishPhysics
{
	class ParticleContactGenerator
	{
	public:

		/**Fills the contact list with the generated contacts.
		* @param contact - The list of contacts to be filled.
		* @Param limit - The maximum number of contacts that can be filled.
		* @return - An unsigned int that represents the number of contacts that were filled.*/
		virtual std::uint32_t AddContact(class ParticleContact *contact, std::uint32_t limit) const = 0;
	};
}