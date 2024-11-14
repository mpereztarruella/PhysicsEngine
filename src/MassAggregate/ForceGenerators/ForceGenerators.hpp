/**
 * @file ForceGenerators.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief This file would contain the different force generators headers for the physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-01-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "ForceGeneratorBase.hpp"

namespace Ocacho::Physics::MassAggregate
{
	//=========================================================================
	//GRAVITY FORCE GENERATOR
	//=========================================================================
	
	/**
	 * @brief Force generator for the Gravity force.
	 * 
	 */
	class GravityForceGenerator : public ForceGenerator<GravityForceGenerator>
	{
		private:
			const Vector3 gravity {0.0f, -20.0f, 0.0f};

		public:
			/**
			 * @brief Adds gravity force to the particle.
			 * 
			 * @param p_particle Particle to apply the gravity.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(Particle& p_particle, const float deltaTime) const noexcept;
	};

	//=========================================================================
	//DRAG FORCE GENERATOR
	//=========================================================================
	
	/**
	 * @brief Force generator for the Drag force.
	 * 
	 */
	class DragForceGenerator : public ForceGenerator<DragForceGenerator>
	{
		private:
			//TODO[Otto]: Ajustar estos valores para hacer un drag bien guapote
			const float k1 {1.0f};
			const float k2 {0.5f};

		public:
			/**
			 * @brief Adds drag force to the particle.
			 * 
			 * @param p_particle Particle to apply the drag force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(Particle& p_particle, const float deltaTime) const noexcept;
	};

	//=========================================================================
	//SPRING FORCE GENERATORS
	//=========================================================================
	
	/**
	 * @brief Force generator to simulate a spring force that connects two particles.
	 * 
	 */
	class SpringForceGenerator : public ForceGenerator<SpringForceGenerator>
	{
		private:
			//Holds the spring constant
			const float springConstant;
			//Holds the rest length of the spring
			const float restLength;
			//Holds the particle at the other side of the spring, to which the force is not applied
			const Particle& otherParticle;

		public:
			explicit SpringForceGenerator(float p_springConstant, float p_restLength,
										  const Particle& p_otherParticle) noexcept;
			/**
			 * @brief Adds spring force to the particle.
			 * 
			 * @param p_particle Particle to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(Particle& p_particle, const float deltaTime) const noexcept;
	};

	/**
	 * @brief Force generator to simulate a spring force with a particle connected to an anchored position.
	 * 
	 */
	class AnchoredSpringForceGenerator : public ForceGenerator<AnchoredSpringForceGenerator>
	{
		private:
			//Holds the spring constant
			const float springConstant;
			//Holds the rest length of the spring
			const float restLength;
			//Holds the position at the other side of the spring
			const Vector3 anchoredPosition;

		public:
			explicit AnchoredSpringForceGenerator(const float p_springConstant, const float p_restLength,
										  		  const Vector3 p_anchoredPosition) noexcept;
			/**
			 * @brief Adds spring force to the particle.
			 * 
			 * @param p_particle Particle to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(Particle& p_particle, const float deltaTime) const noexcept;
	};

	/**
	 * @brief Force generator to simulate an elastic bungee that connects two particles.
	 * 
	 */
	class ElasticBungeeForceGenerator : public ForceGenerator<ElasticBungeeForceGenerator>
	{
		private:
			//Holds the spring constant
			const float springConstant;
			//Holds the rest length of the spring
			const float restLength;
			//Holds the particle at the other side of the spring, to which the force is not applied
			const Particle& otherParticle;

		public:
			explicit ElasticBungeeForceGenerator(const float p_springConstant, const float p_restLength,
												const Particle& p_otherParticle) noexcept;
			/**
			 * @brief Adds spring force to the particle.
			 * 
			 * @param p_particle Particle to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(Particle& p_particle, const float deltaTime) const noexcept;
	};

	//TODO[Otto]: Aqui iria generador de fuerza de flotabilidad, es un poco movida asi que lo dejo para mas adelante
	//pero se hara porque esta guapisimo joder.

	//=========================================================================
	//FAKE SPRING FORCE GENERATOR
	//=========================================================================

	class FakeSpringForceGenerator : public ForceGenerator<FakeSpringForceGenerator>
	{
		private:
			//Holds the spring constant
			const float springConstant;
			//Holds the damping for the oscillation of the spring
			const float damping;
			//Holds the position at the other side of the spring
			const Vector3 anchoredPosition;

		public:
			explicit FakeSpringForceGenerator(const float p_springConstant, const float p_damping,
											  const Vector3 p_anchoredPosition) noexcept;

			/**
			 * @brief Adds spring force to the particle.
			 * 
			 * @param p_particle Particle to apply the spring force.
			 * @param deltaTime Ellapsed time calculated in base of time not frames.
			 */
			void updateForce(Particle& p_particle, const float deltaTime) const noexcept;
	};
}//namespace Ocacho::Physics::MassAggregate

#include "ForceGenerators.tpp"
	