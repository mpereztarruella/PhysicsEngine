/**
 * @file Contact.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief This file would contain the contact class that contains the information to resolve collisions detected by the collision detector.
 * 
 * 
 * @version 0.1
 * @date 2022-01-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "MassAggregate/Particle.hpp"

namespace Ocacho::Physics::MassAggregate
{
	class Contact
	{
		public:
			
			//Holds the particles that are involved in the contact.
			//The second will be NULL for contacts with the scenery.
			Particle* particles_[2];

			//Holds the restitution coefficient for the contact.
			float restitution_;

			//Holds the normal vector for the direction of the contact.
			Vector3 contactNormal_;

			//Holds depth of penetration for the contact.
			float penetration_;

			//Holds the amount of movement for each particle at interpenetration resolution.
			Vector3 particleMovement_[2];

		public:
			//=========================================================================
			//PUBLIC METHODS
			//=========================================================================

			/**
			 * @brief Resolves the contact, for velocity and interpenetration
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void resolve(const float p_deltaTime) noexcept;

			/**
			 * @brief Calculates the separating velocity for the particles in the contact.
			 * 
			 * @return float The separating velocity.
			 */
			float calculateSeparatingVelocity() const noexcept;

			/**
			 * @brief Get the Movement For Particle object
			 * 
			 * @param p_particle The reference of the particle to check.
			 * @return Vector3 The amount of movement to modify.
			 */
			Vector3 getMovementForParticle(Particle& p_particle) const noexcept;

		private:
			//=========================================================================
			//PRIVATE METHODS
			//=========================================================================

			/**
			 * @brief Checks the velocity build-up due to acceleration only.
			 * 
			 * @param p_newSepVel Separating velocity for the contact.
			 * @param p_deltaTime Elapsed time between frames.
			 * @return float The new value for the newSepVel.
			 */
			float velocityDueToAccel(float p_newSepVel, const float p_deltaTime) const noexcept;

			/**
			 * @brief Resolves the velocity for this contact, the calculus is made from the
			 * 		  first particle's perspective.
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void resolveVelocity(const float p_deltaTime) const noexcept;

			/**
			 * @brief Resolves the interpenetration for this contact.
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void resolveInterpenetration(const float p_deltaTime) noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate