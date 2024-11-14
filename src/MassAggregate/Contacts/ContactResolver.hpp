/**
 * @file ContactResolver.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief This file would contain the contact resolver algorithm for the contacts of the physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-01-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "Contact.hpp"

namespace Ocacho::Physics::MassAggregate
{
	class ContactResolver
	{
		private:
			//Holds the maximum number of iterations that our algorithm will make (at least equal to the number of the contacts in the physics engine)
			unsigned iterations_;

			//Holds the used iterations in each update of the Resolver
			unsigned usedIterations_;

		public:
			ContactResolver(unsigned p_iterations) noexcept;

			//=========================================================================
			//PUBLIC METHODS
			//=========================================================================

			/**
			 * @brief Sets the maximum number of iterations for the contact resolver algorithm.
			 * 
			 * @param p_iterations number of maximum iterations to set.
			 */
			void setIterations(unsigned p_iterations) noexcept;

			/**
			 * @brief Updates the interpenetration for the contacts after have resolved the most relevant contact
			 * 
			 * @param p_contactList Array of contacts to update the penetration.
			 * @param p_maxIndex The index of the most relevant contact in the contact array.
			 */
			void updateInterpenetrations(std::vector<Contact>& p_contactList, unsigned p_maxIndex) const noexcept;
			
			/**
			 * @brief For a set of particle contacts given, resolves the velocity and interpenetration.
			 * 
			 * @param p_contactList Array of contacts to resolve.
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void resolveContacts(std::vector<Contact>& p_contactList, const float p_deltaTime) noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate