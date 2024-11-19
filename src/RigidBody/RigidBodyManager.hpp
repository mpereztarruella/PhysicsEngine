/**
 * @file ParticleManager.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief This class contains all the data that the manager will need to create and update the rigidbodies with the physics engine.
 *
 *
 * @version 0.1
 * @date 2022-01-31
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "Utility/MetaClasses.hpp"
#include "Physics.hpp"
#include "RigidBody/ForceGenerators/ForceGeneratorBaseRB.hpp"

namespace Ocacho::Physics::RigidBody
{
	enum ColliderTypeEnum
	{
		None = -1,
		Sphere,
		Cube,
		Count
	};

	template <typename ForceList>
	class RigidBodyManager
	{
		private:
			using forceRegistryT = typename Ocacho::Meta::replace<ForceRegistryRB, ForceList>::type;

			//Holds the reference to the RigidBodies
			std::vector<RigidBody*> rigidBodies_;

			//Holds the force registry for the engine
			forceRegistryT forceReg_;

		private:
			//=========================================================================
			//PRIVATE METHODS
			//=========================================================================
			/**
			 * @brief Calls the integrator for all the RigidBody in the RigidBody manager.
			 *
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void integrateRigidBodies(const float p_deltaTime) noexcept;

		public:
			RigidBodyManager() noexcept = default;

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
			 * @brief Call this method to add a new RigidBody to the manager.
			 *
			 * @param p_particle Reference to the RigidBody to be added.
			 */
			void addRigidBody(RigidBody& p_rigidBody) noexcept;

			/**
			 * @brief Call this method to add a new Force Registration that links a force generator
			 * with a particle.
			 *
			 * @param p_rigidBody Reference to the RigidBody which will be linked to a force generator.
			 * @param p_forceGen Reference to the force generator to link the RigidBody with.
			 */
			void addForceRegistration(RigidBody& p_rigidBody, auto& p_forceGen) noexcept;
	};
}//namespace Ocacho::Physics::RigidBody

#include "RigidBodyManager.tpp"