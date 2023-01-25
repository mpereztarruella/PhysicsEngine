#include "Contact.hpp"

namespace Ocacho::Physics::MassAggregate
{
	void
	Contact::resolve(const float p_deltaTime) noexcept
	{
		resolveVelocity(p_deltaTime);
		resolveInterpenetration(p_deltaTime);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	Contact::calculateSeparatingVelocity() const noexcept
	{
		Vector3 separatingVelocity;

		separatingVelocity = particles_[0]->velocity;

		if(particles_[1])
			separatingVelocity -= particles_[1]->velocity;
		
		float vs = glm::dot(separatingVelocity, contactNormal_);

		return vs;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	Vector3
	Contact::getMovementForParticle(Particle& p_particle) const noexcept
	{
		if(*particles_[0] == p_particle) return particleMovement_[0];
		else if(*particles_[1] == p_particle) return particleMovement_[1];

		return Vector3{};
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	float
	Contact::velocityDueToAccel(float p_newSepVel, const float p_deltaTime) const noexcept
	{
		Vector3 accCausedVel = particles_[0]->acceleration;
		
		if(particles_[1])
			accCausedVel -= particles_[1]->acceleration;

		float accCausedSepVel = glm::dot(accCausedVel, contactNormal_) * p_deltaTime;

		if(accCausedSepVel < 0)
		{
			p_newSepVel += restitution_ * accCausedSepVel;

			//Check that we don't remove more than necessary
			if(p_newSepVel < 0)
				p_newSepVel = 0;
		}

		return p_newSepVel;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::resolveVelocity(const float p_deltaTime) const noexcept
	{
		float separatingVelocity = calculateSeparatingVelocity();

		//If separatingVelocity > 0 it means that the partices are either stationary or separating
		if(separatingVelocity > 0) return;

		float newSepVelocity = -separatingVelocity * restitution_;

		//Check the velocity buil-up by the acceleration only
		newSepVelocity = velocityDueToAccel(newSepVelocity, p_deltaTime);

		float deltaVelocity = newSepVelocity - separatingVelocity;

		float totalInvMass = particles_[0]->inverseMass;
		
		if(particles_[1]) totalInvMass += particles_[1]->inverseMass;

		//The total of mass is infinite
		if(totalInvMass <= 0) return;

		//Amount of impulse per unit of inverse mass
		float impulse = deltaVelocity / totalInvMass;

		//Find the impulse to apply per unit of inverse mass
		Vector3 impulsePerInvMass = contactNormal_ * impulse;

		particles_[0]->velocity += impulsePerInvMass * particles_[0]->inverseMass;

		//For the second particle the impulse to apply would be the inverse than for the first particle
		if(particles_[1])
			particles_[1]->velocity += impulsePerInvMass * -particles_[1]->inverseMass;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::resolveInterpenetration(const float p_deltaTime) noexcept
	{
		if(penetration_ <= 0) return;

		float totalInvMass = particles_[0]->inverseMass;

		if(particles_[1]) totalInvMass += particles_[1]->inverseMass;

		//The total of mass is infinite
		if(totalInvMass <= 0) return;

		//Find the penetration resolution per unit of inverse mass
		Vector3 movePerInvMass = contactNormal_ * (penetration_ / totalInvMass);

		particleMovement_[0] = movePerInvMass * particles_[0]->inverseMass;

		if(particles_[1])
			particleMovement_[1] = movePerInvMass * -particles_[1]->inverseMass;
		else
			particleMovement_[1] = Vector3(0.0f, 0.0f, 0.0f);

		particles_[0]->position += particleMovement_[0];

		if(particles_[1])
			particles_[1]->position += particleMovement_[1];
	}
}//namespace Ocacho::Physics::MassAggregate