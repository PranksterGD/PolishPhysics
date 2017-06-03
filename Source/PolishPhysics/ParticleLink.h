#pragma once
#include "ParticleContactGenerator.h"
#include "Precision.h"
namespace PolishPhysics
{
	/**Links two particles together, generating a contact if they violate the constraints of their link. */
	class ParticleLink : public ParticleContactGenerator
	{
	public: 
		
		/**Holds the two particles that are connected by this link. */
		class Particle* mParticles[2];

	protected:

		/**Returns the current length of the link.
		* @return - A precision that represents the current length of the link.*/
		Precision GetCurrentLength() const;

	public:

		/**Fills the contact list with the generated contacts.
		* @param contact - The list of contacts to be filled.
		* @Param limit - The maximum number of contacts that can be filled.
		* @return - An unsigned int that represents the number of contacts that were filled.*/
		virtual std::uint32_t AddContact(class ParticleContact *contact, std::uint32_t limit) const = 0;
	};
}