/**
 * @file ParticleManager.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This class contains all the data that the manager will need to update the particles with the physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-01-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "Utility/TypeAliases.hpp"

#include "Physics.hpp"
#include "MassAggregate/ForceGenerators/ForceGeneratorBase.hpp"
#include "MassAggregate/Contacts/ContactResolver.hpp"
#include "MassAggregate/Contacts/ContactGenerator.hpp"

namespace Ocacho::Physics::MassAggregate
{
	/**
	 * @brief Use this struct to create the two lists of parameters we'll need for the ParticleManager.
	 * The first list for the force registry that will contain all the force generator types.
	 * The second list for the contact generator vector that will contain all the contact generator types.
	 * 
	 * @tparam Ts The list of types that will contain the Typelist.
	 */
	template<typename... Ts>
	struct Typelist {};

	namespace
	{
		template <template <typename...> class New, typename List>
		struct replace {};
		template <template <typename...> class New, typename... Ts>
		struct replace<New, Typelist<Ts...>>
		{
			using type = New<Ts...>;
		};
	}

	template <typename ForceList, typename ContactGenList>
	class ParticleManager
	{
		private:
			using forceRegType = typename replace<ForceRegistry, ForceList>::type;
			using contactGenVariantType = typename replace<std::variant, ContactGenList>::type;

			//Holds the reference to the particles
			std::vector<Particle*> particles_;

			//Holds the force registry for the engine
			forceRegType forceReg_;

			//Holds the contact resolver
			ContactResolver contactRes_;

			//Holds the list of contacts
			std::vector<Contact> contactList_;

			//Holds the contact generators for the manager
			std::vector<contactGenVariantType> contactGenVector_;

		private:
			//=========================================================================
			//PRIVATE METHODS
			//=========================================================================

			/**
			 * @brief Calls the integrator for all the particles in the particle manager.
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void integrateParticles(const float p_deltaTime) noexcept;

			/**
			 * @brief Calls the addContact method for all the contact generators of the particle manager.
			 * 
			 * @return unsigned The number of contacts generated each frame.
			 */
			unsigned generateContacts() noexcept;
		
		public:
			//TODO[Otto]: De momento las iteraciones van a ser siempre dependientes de la cantidad de
			//contactos generados, en un futuro meter la opcion de que sean fijas en todos los frames.
			ParticleManager(unsigned p_maxContacts, unsigned p_iterations = 0) noexcept;

			//=========================================================================
			//PUBLIC METHODS
			//=========================================================================

			/**
			 * @brief Call this method at start of each frame to reset the values of the manager.
			 * 
			 */
			void reset() noexcept;

			/**
			 * @brief Updates the data for the physics engine contained in the manager.
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void runPhysics(const float p_deltaTime) noexcept;

			//=========================================================================
			//METHODS TO ADD DATA
			//=========================================================================

			/**
			 * @brief Call this method to add a new particle to the manager.
			 * 
			 * @param p_particle Reference to the particle that we want to add.
			 */
			void addParticle(Particle& p_particle) noexcept;

			/**
			 * @brief Call this method to add a new contact generator to the manager.
			 * 
			 * @param p_contactGen Contact generator passed by parameter, it will generate 
			 * 	a copy so it can be deleted in other sites.
			 */
			void addContactGenerator(auto p_contactGen) noexcept;

			/**
			 * @brief Call this method to add a new Force Registration that links a force generator
			 * with a particle.
			 * 
			 * @param p_particle Reference to the particle to which we want to link with a force generator.
			 * @param p_forceGen Reference to the force generator that we want to link the particle with.
			 */
			void addForceRegistration(Particle& p_particle, auto& p_forceGen) noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate

#include "ParticleManager.tpp"
