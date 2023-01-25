/**
 * @file ForceGeneratorsRB.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file would contain the different force generators headers for RigidBody the physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "ForceGeneratorBaseRB.hpp"
#include <Physics.hpp>

namespace Ocacho::Physics::RigidBody
{
	//=========================================================================
	//GRAVITY FORCE GENERATOR
	//=========================================================================
	
	/**
	 * @brief Force generator for the Gravity force.
	 * 
	 */
	class GravityForceGeneratorRB : public ForceGeneratorRB<GravityForceGeneratorRB>
	{
		private:
			const Vector3 gravity_ {0.0f, -20.0f, 0.0f};

		public:
			/**
			 * @brief Adds gravity force to the RigidBody.
			 * 
			 * @param p_rigidBody RigidBody to apply the gravity.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(RigidBody& p_rigidBody, const float deltaTime) const noexcept;
	};

	//=========================================================================
	//DRAG FORCE GENERATOR
	//=========================================================================
	
	/**
	 * @brief Force generator for the Drag force.
	 * 
	 */
	class DragForceGeneratorRB : public ForceGeneratorRB<DragForceGeneratorRB>
	{
		private:
			//TODO[Otto]: Ajustar estos valores para hacer un drag bien guapote
			const float k1_ {1.0f};
			const float k2_ {0.5f};

		public:
			/**
			 * @brief Adds drag force to the RigidBody.
			 * 
			 * @param p_rigidBody RigidBody to apply the drag force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(RigidBody& p_rigidBody, const float deltaTime) const noexcept;
	};

	//=========================================================================
	//SPRING FORCE GENERATORS
	//=========================================================================
	
	/**
	 * @brief Force generator to simulate a spring force that connects two RigidBodies.
	 * 
	 */
	class SpringForceGeneratorRB : public ForceGeneratorRB<SpringForceGeneratorRB>
	{
		private:
			//Holds the point in local space of the coordinates where the Spring is attached to the RigidBody.
			Vector3 connectPoint_;
			//Holds the spring constant.
			const float springConstant_;
			//Holds the rest length of the spring.
			const float restLength_;
			//Holds the point in local space where the spring is attached to the other RigidBody.
			Vector3 otherConnectPoint_;
			//Holds the RigidBody at the other side of the spring, to which the force is not applied.
			const RigidBody& otherRigidBody_;

		public:
			explicit SpringForceGeneratorRB(Vector3 p_localConnPt
				, float p_springConstant
				, float p_restLength
				, Vector3 p_otherConnPt
				, const RigidBody& p_otherRigidBody) noexcept;
			/**
			 * @brief Adds spring force to the RigidBody.
			 * 
			 * @param p_rigidBody RigidBody to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(RigidBody& p_rigidBody, const float deltaTime) noexcept;
	};

	/**
	 * @brief Force generator to simulate a spring force with a RigidBody connected to an anchored position.
	 * 
	 */
	class AnchoredSpringForceGeneratorRB : public ForceGeneratorRB<AnchoredSpringForceGeneratorRB>
	{
		private:
			//Holds the point in local space of the coordinates where the Spring is attached to the RigidBody.
			Vector3 connectPoint_;
			//Holds the spring constant.
			const float springConstant_;
			//Holds the rest length of the spring.
			const float restLength_;
			//Holds the position at the other side of the spring in world space.
			const Vector3 anchoredPosition_;

		public:
			explicit AnchoredSpringForceGeneratorRB(Vector3 p_localConnPt
				, const float p_springConstant
				, const float p_restLength
				, const Vector3 p_anchoredPosition) noexcept;
			/**
			 * @brief Adds spring force to the RigidBody.
			 * 
			 * @param p_rigidBody RigidBody to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(RigidBody& p_rigidBody, const float deltaTime) noexcept;
	};

	/**
	 * @brief Force generator to simulate an elastic bungee that connects two particles.
	 * 
	 */
	class ElasticBungeeForceGeneratorRB : public ForceGeneratorRB<ElasticBungeeForceGeneratorRB>
	{
		private:
			//Holds the point in local space where the spring is attached to the RigidBody.
			Vector3 connectPoint_;
			//Holds the spring constant.
			const float springConstant_;
			//Holds the rest length of the spring.
			const float restLength_;
			//Holds the point in local space where the spring is attached to the other RigidBody.
			Vector3 otherConnectPoint_;
			//Holds the particle at the other side of the spring, to which the force is not applied.
			const RigidBody& otherRigidBody_;

		public:
			explicit ElasticBungeeForceGeneratorRB(Vector3 p_localConnPt
				, float p_springConstant
				, float p_restLength
				, Vector3 p_otherConnPt
				, const RigidBody& p_otherRigidBody) noexcept;
			/**
			 * @brief Adds spring force to the RigidBody.
			 * 
			 * @param p_rigidBody RigidBody to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(RigidBody& p_rigidBody, const float deltaTime) noexcept;
	};

	//TODO[Otto]: Aqui iria generador de fuerza de flotabilidad, es un poco movida asi que lo dejo para mas adelante
	//pero se hara porque esta guapisimo joder.

	//=========================================================================
	//FAKE SPRING FORCE GENERATOR
	//=========================================================================

	class FakeSpringForceGeneratorRB : public ForceGeneratorRB<FakeSpringForceGeneratorRB>
	{
		private:
			//Holds the point in local space where the spring is attached to the RigidBody.
			Vector3 connectPoint_;
			//Holds the spring constant.
			const float springConstant_;
			//Holds the damping for the oscillation of the spring.
			const float damping_;
			//Holds the position at the other side of the spring in world space.
			const Vector3 anchoredPosition_;

		public:
			explicit FakeSpringForceGeneratorRB(Vector3 p_localConnPt
				, const float p_springConstant
				, const float p_damping
				, const Vector3 p_anchoredPosition) noexcept;

			/**
			 * @brief Adds spring force to the RigidBody.
			 * 
			 * @param p_rigidBody RigidBody to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(RigidBody& p_rigidBody, const float deltaTime) noexcept;
	};
}//namespace Ocacho::Physics::RigidBody

#include "ForceGeneratorsRB.tpp"
	