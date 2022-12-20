/**
 * @file Contacts.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief Contains the structure of data and the different methods to create the contacts
 * for the collision system.
 * 
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 */

#pragma once

#include "Utility/TypeAliases.hpp"

#include "Physics/RigidBody/RigidBody.hpp"

namespace Ocacho::Physics::RigidBody
{
	struct Contact
	{
		//Holds the two bodies involved in the contact
		RigidBody* bodies_[2];
		//Holds the friction coefficient for the contact
		float friction_				{.5f};
		//Holds the restitution coefficient for the contact
		float restitution_			{.5f};
		//Holds the position of the contact in world space.
		Vector3 contactPoint_		{0, 0, 0};
		//Holds the direction of the contact in world space.
		Vector3 contactNormal_		{0, 0, 0};
		//Holds the amount of interpenetration at the contact point.
		float penetration_			{0};

		/**
		 * @brief Method to set the body, friction and restitution data of the contact.
		 * 
		 * @param p_first RigidBody pointer to be set to the first index of bodies_.
		 * @param p_second RigidBody pointer to be set to the second index of bodies_.
		 * @param p_friction Friction coefficient to set to the contact.
		 * @param p_restitution Restitution coefficient to set to the contact.
		 */
		void setBodyData(RigidBody* p_first, RigidBody* p_second
			, float p_friction, float p_restitution)
		{
			bodies_[0] = p_first;
			bodies_[1] = p_second;
			friction_ = p_friction;
			restitution_ = p_restitution;
		}
	};
}//namespace Ocacho::Physics::RigidBody