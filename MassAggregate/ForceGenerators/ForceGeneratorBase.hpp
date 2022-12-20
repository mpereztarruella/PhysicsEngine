/**
 * @file ForceGeneratorBase.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file would contain the base force generator header for the physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-01-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "Utility/TypeAliases.hpp"
#include "Debug/Assertion.hpp"
#include "Debug/Profiling/Profiling.hpp"

#include "Physics/MassAggregate/Particle.hpp"

namespace Ocacho::Physics::MassAggregate
{
	/**
	 * @brief The main class to create new ForceGenerators
	 * 
	 * @tparam CRTPType The type of the derived class.
	 */
	template<typename CRTPType>
	class ForceGenerator
	{
		private:
			ForceGenerator(){};
			friend CRTPType;
		
		public:
			/**
			 * @brief Main update method that the derived classes will use to apply the force to the particle.
			 * 
			 * @param p_particle The particle to which we are applying the force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(Particle& p_particle, const float deltaTime) const noexcept;
	};

	//=========================================================================
	//FORCE REGISTRATION
	//=========================================================================

	/**
	 * @brief Struct that links a particle with a force generator.
	 * 
	 * @tparam genTypes Possible types for the force generators that will be present on the variant variable. 
	 */
	template<typename... genTypes>
	struct ForceRegistration
	{
		// TODO [Otto]: Por que el auto p_forceGenerator no me crea una copia?
		explicit ForceRegistration(Particle& p_particle, auto p_forceGenerator) noexcept;

		//Copy assignment operator
		ForceRegistration& operator=(const ForceRegistration& p_fReg) noexcept;

		Particle& particle_;
		std::variant<genTypes*...> forceGen_;
	};

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	/**
	 * @brief Class that contains all the relations between particles and force generators to update them.
	 * 
	 * @tparam genTypes Possible types for the force generators to contain the ForceRegistrations.
	 */
	template<typename... genTypes>
	class ForceRegistry
	{
		protected:
			std::vector<ForceRegistration<genTypes...>> registrations_;

		public:
			/**
			 * @brief Creates a registry that links a particle with a force generator.
			 * 
			 * @param p_particle Particle to link with a force generator.
			 * @param p_fGen Force generator that will be applied to the particle.
			 */
			void add(Particle& p_particle, auto& p_fGen) noexcept;
			
			/**
			 * @brief Remove a registry to unlink a particle to a force generator.
			 * 
			 */
			template<typename generatorType>
			void remove(Particle& p_particle) noexcept;

			/**
			 * @brief Removes all the registries for the force generators and the particles.
			 * 
			 */
			void clear() noexcept;

			/**
			 * @brief 
			 * 
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForces(const float deltaTime) noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate

#include "ForceGeneratorBase.tpp"