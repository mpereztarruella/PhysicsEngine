/**
 * @file ForceGeneratorBaseRB.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file would contain the base force generator header for the RigidBody physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "Utility/TypeAliases.hpp"

#include "RigidBody/RigidBody.hpp"

namespace Ocacho::Physics::RigidBody
{
	/**
	 * @brief The main class to create new ForceGenerators
	 * 
	 * @tparam CRTPType The type of the derived class.
	 */
	template<typename CRTPType>
	class ForceGeneratorRB
	{
		private:
			ForceGeneratorRB(){};
			friend CRTPType;
		
		public:
			/**
			 * @brief Main update method that the derived classes will use to apply the force to the RidigBody.
			 * 
			 * @param p_rigidBody The RidigBody to which we are applying the force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(RigidBody& p_rigidBody, const float deltaTime) noexcept;
	};

	//=========================================================================
	//FORCE REGISTRATION
	//=========================================================================

	/**
	 * @brief Struct that links a RidigBody with a force generator.
	 * 
	 * @tparam genTypes Possible types for the force generators that will be present on the variant variable. 
	 */
	template<typename... genTypes>
	struct ForceRegistrationRB
	{
		// TODO [Otto]: Por que el auto p_forceGenerator no me crea una copia?
		explicit ForceRegistrationRB(RigidBody& p_rigidBody, auto p_forceGenerator) noexcept;

		//Copy assignment operator
		ForceRegistrationRB& operator=(const ForceRegistrationRB& p_fReg) noexcept;

		RigidBody& rigidBody_;
		std::variant<genTypes*...> forceGen_;
	};

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	/**
	 * @brief Class that contains all the relations between RigidBodies and force generators to update them.
	 * 
	 * @tparam genTypes Possible types for the force generators to contain the ForceRegistrations.
	 */
	template<typename... genTypes>
	class ForceRegistryRB
	{
		protected:
			std::vector<ForceRegistrationRB<genTypes...>> registrations_;

		public:
			/**
			 * @brief Creates a registry that links a RigidBody with a force generator.
			 * 
			 * @param p_rigidBody RigidBody to link with a force generator.
			 * @param p_fGen Force generator that will be applied to the RigidBody.
			 */
			void add(RigidBody& p_rigidBody, auto& p_fGen) noexcept;
			
			/**
			 * @brief Remove a registry to unlink a RigidBody to a force generator.
			 * 
			 */
			template<typename generatorType>
			void remove(RigidBody& p_rigidBody) noexcept;

			/**
			 * @brief Removes all the registries for the force generators and the RigidBodies.
			 * 
			 */
			void clear() noexcept;

			/**
			 * @brief Updates the forces for all the registrations in the registration vector.
			 * 
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForces(const float deltaTime) noexcept;
	};
}//namespace Ocacho::Physics::RigidBody

#include "ForceGeneratorBaseRB.tpp"