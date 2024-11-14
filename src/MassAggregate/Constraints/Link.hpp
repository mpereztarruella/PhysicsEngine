/**
 * @file Link.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief This file would contain the link class to be derived by others to create hard constraints.
 * 
 * 
 * @version 0.1
 * @date 2022-01-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "MassAggregate/Contacts/ContactGenerator.hpp"

namespace Ocacho::Physics::MassAggregate
{
	template<typename CRTPType>
	class Link : ContactGenerator<Link<CRTPType>>
	{
		private:
			Link();
			friend CRTPType;
		
		public:
			//Holds the particles which will be linked by a constraint.
			Particle* particles_[2] {};

		private:
			//=========================================================================
			//PRIVATE METHODS
			//=========================================================================
			
			/**
			 * @brief Calculates the current length of the link.
			 * 
			 * @return float returns the current length.
			 */
			float currentLength() const noexcept;

			/**
			 * @brief Adds a contact to maintain a constraint if necessary.
			 * 
			 * @param p_contactList Vector reference to the list of contacts.
			 * @return unsigned Zero if didn't produced any contact, 1 if the operation has produced a contact.
			 */
			unsigned addContact(std::vector<Contact>& p_contactList) const noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate

#include "Link.tpp"