#include "RigidBody.hpp"

namespace Ocacho::Physics::RigidBody
{
	RigidBody&
	RigidBody::operator=(const RigidBody& p_rigidBody) noexcept
	{
		if(this == &p_rigidBody)
			return *this;
		
		mass = p_rigidBody.mass;
		inverseMass = p_rigidBody.inverseMass;
		
		position = p_rigidBody.position;
		velocity = p_rigidBody.velocity;
		acceleration = p_rigidBody.acceleration;
		lastFrameAccel = p_rigidBody.lastFrameAccel;
		orientation = p_rigidBody.orientation;
		rotation = p_rigidBody.rotation;
		forceAccum = p_rigidBody.forceAccum;
		torqueAccum = p_rigidBody.torqueAccum;
		transformMatrix = p_rigidBody.transformMatrix;
		inverseInertiaTensor = p_rigidBody.inverseInertiaTensor;
		invInertiaTensorWorld = p_rigidBody.invInertiaTensorWorld;

		return *this;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint8_t
	RigidBody::operator==(const RigidBody& p_rigidBody) const noexcept
	{
		if(p_rigidBody.mass != mass)									return 0;

		if(p_rigidBody.inverseMass != inverseMass)						return 0;

		if(p_rigidBody.position != position)							return 0;

		if(p_rigidBody.velocity != velocity)							return 0;

		if(p_rigidBody.acceleration != acceleration)					return 0;

		if(p_rigidBody.lastFrameAccel != lastFrameAccel)				return 0;

		if(p_rigidBody.orientation != orientation)						return 0;

		if(p_rigidBody.rotation != rotation)							return 0;

		if(p_rigidBody.forceAccum != forceAccum)						return 0;

		if(p_rigidBody.torqueAccum != torqueAccum)						return 0;

		if(p_rigidBody.transformMatrix != transformMatrix)				return 0;

		if(p_rigidBody.inverseInertiaTensor != inverseInertiaTensor)	return 0;

		if(p_rigidBody.invInertiaTensorWorld != invInertiaTensorWorld)	return 0;

		return 1;
	}

}//namespace Ocacho::Physics::RigidBody
