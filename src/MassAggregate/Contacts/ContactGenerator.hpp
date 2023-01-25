/**
 * @file ContactGenerator.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file contains the base class to be derived to create new contact generators.
 * 
 * 
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "Contact.hpp"

namespace Ocacho::Physics::MassAggregate
{
	template<typename CRTPType>
	class ContactGenerator
	{
		private:
			ContactGenerator() {}
			friend CRTPType;
		
		public:
			//=========================================================================
			//PUBLIC METHODS
			//=========================================================================
			
			/**
			 * @brief Adds a contact to maintain a constraint if necessary.
			 * 
			 * @param p_contactList Vector reference to the list of contacts.
			 * @return unsigned Zero if didn't produced any contact, 1 if the operation has produced a contact.
			 */
			unsigned addContact(std::vector<Contact>& p_contactList) const noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate

#include "ContactGenerator.hpp"