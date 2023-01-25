#include "Contacts.hpp"
#include "Physics.hpp"
#include <cassert>

//TEST
#include <glm/gtx/io.hpp>
//TEST

namespace Ocacho::Physics::RigidBody
{
	//=============================================================================
	//CONTACT
	//=============================================================================
	void
	Contact::calculateInternals(const float p_deltaTime) noexcept
	{
		if(!bodies_[0]) swapBodies();
		assert(bodies_[0]);

		calculateContactBasis();
		
		//Store the relative position of the contact to each body
		relativeContactPosition_[0] = contactPoint_ - bodies_[0]->position;
		if(bodies_[1]) relativeContactPosition_[1] = contactPoint_ - bodies_[1]->position;

		//Calculate the relative velocities of the bodies at the contact point
		contactVelocity_ = calculateLocalVelocity(0, p_deltaTime);
		if(bodies_[1]) contactVelocity_ -= calculateLocalVelocity(1, p_deltaTime);

		calculateDesiredDeltaVelocity(p_deltaTime);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::matchAwakeState() noexcept
	{
		if(!bodies_[1]) return;

		bool body_0_Awake = bodies_[0]->isAwake_;
		bool body_1_Awake = bodies_[1]->isAwake_;

		if(body_0_Awake ^ body_1_Awake)
		{
			if(body_0_Awake) bodies_[1]->isAwake_ = true;
			else			 bodies_[0]->isAwake_ = true;
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::swapBodies() noexcept
	{
		contactNormal_ *= -1;

		RigidBody* aux = bodies_[0];
		bodies_[0] = bodies_[1];
		bodies_[1] = aux;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::calculateDesiredDeltaVelocity(const float p_deltaTime) noexcept
	{
		float accVelocity = 0;

		if(bodies_[0]->isAwake_)
			accVelocity += glm::dot( (bodies_[0]->lastFrameAccel * p_deltaTime), contactNormal_);
		
		if(bodies_[1] && bodies_[1]->isAwake_)
			accVelocity -= glm::dot( (bodies_[1]->lastFrameAccel * p_deltaTime), contactNormal_);
		
		float thisRestitution = restitution_;

		if(std::abs(contactVelocity_.x) < velocityLimit_)
			thisRestitution = 0.f;
		
		desiredDeltaVelocity_ = -contactVelocity_.x - thisRestitution * (contactVelocity_.x - accVelocity);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	Vector3
	Contact::calculateLocalVelocity(uint8_t p_index, const float p_deltaTime) noexcept
	{
		RigidBody* thisBody = bodies_[p_index];

		//Calculate the velocity of the contact point
		Vector3 velocity = glm::cross(thisBody->rotation, relativeContactPosition_[p_index]);
		velocity += thisBody->velocity;

		//Transform the velocity to contact-coordinates
		Vector3 contactVelocity = glm::inverse(contactToWorld_) * velocity;

		//Calculate the velocity due to the forces and get it in contact-coordinates
		Vector3 accVelocity = thisBody->lastFrameAccel * p_deltaTime;
		accVelocity = glm::inverse(contactToWorld_) * accVelocity;
		//Ignore the velocity due to forces in the contact normal direction
		accVelocity.x = 0;

		contactVelocity += accVelocity;

		return contactVelocity;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::calculateContactBasis() noexcept
	{
		Vector3 contactTangent[2] {};

		if(std::abs(contactNormal_.x) > std::abs(contactNormal_.y))
		{
			const float s = 1.f / std::sqrt(contactNormal_.z*contactNormal_.z
				+ contactNormal_.x*contactNormal_.x);

			contactTangent[0].x = contactNormal_.z*s;
			contactTangent[0].y = 0;
			contactTangent[0].z = -contactNormal_.x*s;

			contactTangent[1].x = contactNormal_.y*contactTangent[0].x;
			contactTangent[1].y = contactNormal_.z*contactTangent[0].x
				- contactNormal_.x*contactTangent[0].z;
			contactTangent[1].z = contactNormal_.x*contactTangent[0].y;

		}
		else
		{
			const float s = 1.f / std::sqrt(contactNormal_.z*contactNormal_.z
				+ contactNormal_.y*contactNormal_.y);

			contactTangent[0].x = 0;
			contactTangent[0].y = -contactNormal_.z*s;
			contactTangent[0].z = contactNormal_.y*s;

			contactTangent[1].x = contactNormal_.y*contactTangent[0].z
				- contactNormal_.z*contactTangent[0].y;
			contactTangent[1].y = -contactNormal_.x*contactTangent[0].z;
			contactTangent[1].z = contactNormal_.x*contactTangent[0].y;
		}

		contactToWorld_ = Matrix3(contactNormal_, contactTangent[0], contactTangent[1]);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::applyImpulse(const Vector3& p_impulse, RigidBody* p_body
		, Vector3& p_velChange, Vector3 p_rotChange) noexcept
	{

	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	void
	Contact::applyVelocityChange(Vector3 p_velChange[2], Vector3 p_rotChange[2]) noexcept
	{
		Matrix3 invInertiaTensors[2] {};

		invInertiaTensors[0] = bodies_[0]->invInertiaTensorWorld;
		if(bodies_[1]) invInertiaTensors[1] = bodies_[1]->invInertiaTensorWorld;

		Vector3 impulseContact;

		//TODO[Otto]: Aqui ver si se metera el calculo del impulso con friccion
		impulseContact = calculateFrictionlessImpulse(invInertiaTensors);

		Vector3 impulse = contactToWorld_ * impulseContact;

		applyVelocityChangeToBody(0, impulse
			, invInertiaTensors[0], p_velChange[0], p_rotChange[0]);
		
		if(bodies_[1])
		{
			applyVelocityChangeToBody(1, impulse
			, invInertiaTensors[1], p_velChange[1], p_rotChange[1]);
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::applyVelocityChangeToBody(uint8_t p_index, const Vector3& p_impulse
		, const Matrix3 p_invInerTensor, Vector3& p_velChange, Vector3& p_rotChange) noexcept
	{
		Vector3 impulsiveTorque = glm::cross(relativeContactPosition_[p_index], p_impulse);
		
		int mult = (p_index == 0)? 1:-1;

		p_rotChange = p_invInerTensor * impulsiveTorque;
		p_velChange = Vector3();
		p_velChange += p_impulse * (bodies_[p_index]->inverseMass * mult);

		bodies_[p_index]->velocity += p_velChange;
		bodies_[p_index]->rotation += p_rotChange;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	/*void
	Contact::applyPositionChange(Vector3 linearChange[2],
									Vector3 angularChange[2],
									float penetration) noexcept
	{
		const float angularLimit = (float)0.2f;
		float angularMove[2];
		float linearMove[2];

		float totalInertia = 0;
		float linearInertia[2];
		float angularInertia[2];

		// We need to work out the inertia of each object in the direction
		// of the contact normal, due to angular inertia only.
		for (unsigned i = 0; i < 2; i++) if (bodies_[i])
		{
			Matrix3 inverseInertiaTensor = bodies_[i]->invInertiaTensorWorld;

			// Use the same procedure as for calculating frictionless
			// velocity change to work out the angular inertia.
			Vector3 angularInertiaWorld =
				glm::cross(relativeContactPosition_[i], contactNormal_);
			angularInertiaWorld =
				inverseInertiaTensor * angularInertiaWorld;
			angularInertiaWorld =
				glm::cross(angularInertiaWorld, relativeContactPosition_[i]);
			angularInertia[i] =
				glm::dot(angularInertiaWorld, contactNormal_);

			// The linear component is simply the inverse mass
			linearInertia[i] = bodies_[i]->inverseMass;

			// Keep track of the total inertia from all components
			totalInertia += linearInertia[i] + angularInertia[i];

			// We break the loop here so that the totalInertia value is
			// completely calculated (by both iterations) before
			// continuing.
		}

		// Loop through again calculating and applying the changes
		for (unsigned i = 0; i < 2; i++) if (bodies_[i])
		{
			// The linear and angular movements required are in proportion to
			// the two inverse inertias.
			float sign = (i == 0)?1:-1;
			angularMove[i] =
				sign * penetration * (angularInertia[i] / totalInertia);
			linearMove[i] =
				sign * penetration * (linearInertia[i] / totalInertia);

			// To avoid angular projections that are too great (when mass is large
			// but inertia tensor is small) limit the angular move.
			Vector3 projection = relativeContactPosition_[i];
			projection += contactNormal_ * -( glm::dot(relativeContactPosition_[i],contactNormal_) );

			// Use the small angle approximation for the sine of the angle (i.e.
			// the magnitude would be sine(angularLimit) * projection.magnitude
			// but we approximate sine(angularLimit) to angularLimit).
			float maxMagnitude = angularLimit * glm::length(projection);

			if (angularMove[i] < -maxMagnitude)
			{
				float totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = -maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}
			else if (angularMove[i] > maxMagnitude)
			{
				float totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}

			// We have the linear amount of movement required by turning
			// the rigid body (in angularMove[i]). We now need to
			// calculate the desired rotation to achieve that.
			if (angularMove[i] == 0)
			{
				// Easy case - no angular movement means no rotation.
				angularChange[i] = Vector3();
			}
			else
			{
				// Work out the direction we'd like to rotate in.
				Vector3 targetAngularDirection = glm::cross(relativeContactPosition_[i], contactNormal_);

				Matrix3 inverseInertiaTensor = bodies_[i]->invInertiaTensorWorld;

				// Work out the direction we'd need to rotate to achieve that
				angularChange[i] = (inverseInertiaTensor * targetAngularDirection)
					* (angularMove[i] / angularInertia[i]);
			}

			// Velocity change is easier - it is just the linear movement
			// along the contact normal.
			linearChange[i] = contactNormal_ * linearMove[i];
			
			if(penetration == 0.5f)
			{
				std::cout << "p_linChange: " << linearChange[i] << "\n";
			}

			// Now we can start to apply the values we've calculated.
			// Apply the linear movement
			bodies_[i]->position += (contactNormal_ * linearMove[i]);

			// And the change in orientation
			Quaternion aux {angularChange[i]};
			bodies_[i]->orientation *= aux;

			if(!bodies_[i]->isAwake_) CalculateDerivedData(*bodies_[i]);
		}
	}*/


	void
	Contact::applyPositionChange(Vector3 p_linChange[2], Vector3 p_angChange[2]
		, const float p_penetration) noexcept
	{
		float totalInert {0.f}, linearInert[2] {}, angularInert[2] {};

		//Calculate the total inertia of the contact
		for(uint8_t i = 0; i < 2; i++)
		{
			if(bodies_[i])
			{
				Matrix3 inverseInertiaTensor = bodies_[i]->invInertiaTensorWorld;

				Vector3 angularInertiaWorld = glm::cross(relativeContactPosition_[i], contactNormal_);

				angularInertiaWorld = inverseInertiaTensor * angularInertiaWorld;
				angularInertiaWorld = glm::cross(angularInertiaWorld, relativeContactPosition_[i]);
				angularInert[i] = glm::dot(angularInertiaWorld, contactNormal_);
				linearInert[i] = bodies_[i]->inverseMass;

				totalInert += linearInert[i] + angularInert[i];
			}
		}

		for(uint8_t i = 0; i < 2; i++)
		{
			if(bodies_[i])
			{
				applyPositionChangeToBody(i
					, angularInert[i], linearInert[i], totalInert
					, p_linChange[i], p_angChange[i], p_penetration);
			}
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	void
	Contact::applyPositionChangeToBody(uint8_t p_index
		, const float p_angInert, const float p_linInert, const float p_totalInert
		, Vector3& p_linChange, Vector3& p_angChange, const float p_penetration) noexcept
	{
		const float angLimit {.2f};
		float linearMove {}, angularMove {};

		float sign = (p_index == 0)?1:-1;

		angularMove = sign * p_penetration * (p_angInert / p_totalInert);
		linearMove = sign * p_penetration * (p_linInert / p_totalInert);

		Vector3 projection = relativeContactPosition_[p_index];
		float distance = -glm::dot(relativeContactPosition_[p_index], contactNormal_);
		projection += (contactNormal_ *  distance);

		float maxMagnitude = angLimit * glm::length(projection);

		if(angularMove < -maxMagnitude) limitMoveToPositionChange(-maxMagnitude, angularMove, linearMove);
		else if(angularMove > maxMagnitude) limitMoveToPositionChange(maxMagnitude, angularMove, linearMove);

		if(angularMove == 0) p_angChange = Vector3(0,0,0);
		else
		{
			Vector3 targetAngDir = glm::cross(relativeContactPosition_[p_index], contactNormal_);

			Matrix3 invInerTensor;
			invInerTensor = bodies_[p_index]->inverseInertiaTensor;

			p_angChange = (invInerTensor * targetAngDir) * (angularMove / p_angInert);
		}

		p_linChange = contactNormal_ * linearMove;

		if(p_penetration >= 0.2f)
		{
			std::cout << "p_linChange: " << p_linChange << "\n";
			std::cout << "================================\n";
		}

		//TODO[Otto]: Aqui cambiar la multiplicacion por p_linChange
		bodies_[p_index]->position += (contactNormal_ * linearMove);

		//TODO[Otto]: Esto no se si esta bien la verdad (creo que si)
		Quaternion aux {p_angChange};
		bodies_[p_index]->orientation *= aux;

		if(!bodies_[p_index]->isAwake_) CalculateDerivedData(*bodies_[p_index]);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	Contact::limitMoveToPositionChange(const float p_maxMagnitude
		, float& p_angMove, float& p_linMove) const noexcept
	{
		float totalMove = p_angMove + p_linMove;
		p_angMove = p_maxMagnitude;
		p_linMove = totalMove - p_angMove;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	Vector3
	Contact::calculateFrictionlessImpulse(Matrix3 p_invInertiaTensor[2]) noexcept
	{
		Vector3 impulseContact;

		float deltaVel = calculateFrictionlessImpulseBody(0, p_invInertiaTensor[0]);

		if(bodies_[1])
		{
			deltaVel += calculateFrictionlessImpulseBody(1, p_invInertiaTensor[1]);
		}
		
		impulseContact.x = desiredDeltaVelocity_ / deltaVel;
		impulseContact.y = 0;
		impulseContact.z = 0;

		return impulseContact;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	Contact::calculateFrictionlessImpulseBody(uint8_t p_index
		, const Matrix3& p_invInerTensor) noexcept
	{
		Vector3 deltaVelWorld = glm::cross(relativeContactPosition_[p_index], contactNormal_);
		deltaVelWorld = p_invInerTensor * deltaVelWorld;
		deltaVelWorld = glm::cross(deltaVelWorld, relativeContactPosition_[p_index]);

		float deltaVelocity = glm::dot(deltaVelWorld, contactNormal_);

		deltaVelocity += bodies_[p_index]->inverseMass;

		return deltaVelocity;
	}

	//=============================================================================
	//CONTACT RESOLVER
	//=============================================================================

	ContactResolver::ContactResolver(uint32_t p_iterations
		, float velocityEpsilon, float positionEpsilon) noexcept
		: velocityIterations_ {p_iterations}
		, positionIterations_ {p_iterations}
		, velocityEpsilon_ {velocityEpsilon}
		, positionEpsilon_ {positionEpsilon} {}
	
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	ContactResolver::ContactResolver(uint32_t p_velocityIterations, uint32_t p_positionIterations
		, float velocityEpsilon, float positionEpsilon) noexcept
		: velocityIterations_ {p_velocityIterations}
		, positionIterations_ {p_positionIterations}
		, velocityEpsilon_ {velocityEpsilon}
		, positionEpsilon_ {positionEpsilon} {}
	
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	bool
	ContactResolver::isValid() const noexcept
	{
		return (velocityIterations_ > 0)
			&& (positionIterations_ > 0)
			&& (velocityEpsilon_ >= 0.f)
			&& (positionEpsilon_ >= 0.f);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	ContactResolver::setIterations(const uint32_t p_velocityIterations, const uint32_t p_positionIterations) noexcept
	{
		velocityIterations_ = p_velocityIterations;
		positionIterations_ = p_positionIterations;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	ContactResolver::setEpsilon(const float p_velocityEpsilon, const float p_positionEpsilon) noexcept
	{
		velocityEpsilon_ = p_velocityEpsilon;
		positionEpsilon_ = p_positionEpsilon;
	}
}
