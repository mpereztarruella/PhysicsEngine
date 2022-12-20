#include "Particle.hpp"


namespace Ocacho::Physics::MassAggregate
{
	Particle&
	Particle::operator=(const Particle& p_particle) noexcept
	{
		if(this == &p_particle)
			return *this;

		mass = p_particle.mass;
		inverseMass = p_particle.inverseMass;
		
		position = p_particle.position;
		velocity = p_particle.velocity;
		acceleration = p_particle.acceleration;
		forceAccum = p_particle.forceAccum;

		return *this;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint8_t
	Particle::operator==(const Particle& p_particle) const noexcept
	{
		if(p_particle.damping != damping)			return 0;
		
		if(p_particle.mass != mass)					return 0;

		if(p_particle.inverseMass != inverseMass)	return 0;
		
		if(p_particle.position != position)			return 0;

		if(p_particle.velocity != velocity)			return 0;
		
		if(p_particle.acceleration != acceleration)	return 0;

		if(p_particle.forceAccum != forceAccum)		return 0;

		return 1;
	}
}