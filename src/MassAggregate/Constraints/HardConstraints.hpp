/**
 * @file Link.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file would contain the cable class to that creates hard constraints.
 * 
 * 
 * @version 0.1
 * @date 2022-01-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "MassAggregate/Constraints/Link.hpp"

namespace Ocacho::Physics::MassAggregate
{
	//=========================================================================
	//CABLE HARD CONSTRAINT
	//=========================================================================

	class Cable : public Link<Cable>
	{
		public:
			//Holds the max length for the cable.
			float maxLength_;

			//Holds the bounciness of the cable.
			float restitution_;

		public:
			explicit Cable(const float p_maxLength, const float p_restitution) noexcept;

			//=========================================================================
			//PUBLIC METHODS
			//=========================================================================

			/**
			 * @brief Fills the contact given strcuture with the data needed to keep the cable from overextending.
			 * 
			 * @param p_contactList Vector reference to the list of contacts.
			 * @return unsigned Zero if didn't produced any contact, 1 if the operation has produced a contact.
			 */
			unsigned addContact(std::vector<Contact>& p_contactList) const noexcept;
	};

	//=========================================================================
	//ROD HARD CONSTRAINT
	//=========================================================================

	class Rod : public Link<Rod>
	{
		public:
			//Holds the length of the rod.
			float length_;
		
		private:
			//=========================================================================
			//PRIVATE METHODS
			//=========================================================================

			/**
			 * @brief Decides the normal direction for the contact.
			 * 
			 * @param p_contact Reference to auxiliar contact to add the contact normal data.
			 * @param p_currentLen current length of the rod.
			 */
			void evaluateContactNormal(Contact& p_contact, float p_currentLen) const noexcept;

		public:
			explicit Rod(const float p_length) noexcept;

			//=========================================================================
			//PUBLIC METHODS
			//=========================================================================

			/**
			 * @brief Fills the contact given strcuture with the data needed to keep the Rod from extending or compressing.
			 * 
			 * @param p_contactList Vector reference to the list of contacts.
			 * @return unsigned Zero if didn't produced any contact, 1 if the operation has produced a contact.
			 */
			unsigned addContact(std::vector<Contact>& p_contactList) const noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate

#include "HardConstraints.tpp"