/**
 * @file Particle.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file would contain the class with the data needed for physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-01-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "Utility/TypeAliases.hpp"

namespace Ocacho::Physics::MassAggregate
{
	class Particle
	{
		public:
			//Copy assignment operator
			Particle& operator=(const Particle& p_particle) noexcept;

			//Equality operator
			uint8_t operator==(const Particle& p_particle) const noexcept;

			//Value to decrease velocity in each iteration of the physics integrate function
			const float damping		{0.995f};
			//Value that corresponds to the particle's mass
			float mass		{1.0f};
			//The inverse value of the mass, it is used many times in the physics engine
			float inverseMass	{1/mass};

			//Vector that contains the position of the particle
			Vector3 position		{0.0f, 0.0f, 0.0f};
			//Vector that contains the velocity of the particle
			Vector3 velocity		{0.0f, 0.0f, 0.0f};
			//Vector that contains the acceleration of the particle
			Vector3 acceleration	{0.0f, 0.0f, 0.0f};
			//The amount of forces that are applied to the particle
			Vector3 forceAccum		{0.0f, 0.0f, 0.0f};
	};
}//namespace Ocacho::Physics::MassAggregate