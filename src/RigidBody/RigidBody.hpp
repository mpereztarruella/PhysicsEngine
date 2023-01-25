/**
 * @file RigidBody.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file would contain the class with the data needed for physics engine.
 * 
 * 
 * @version 0.2
 * @date 2022-02-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "Utility/TypeAliases.hpp"

namespace Ocacho::Physics::RigidBody
{
	class RigidBody
	{
		public:
			//Copy assignment operator
			RigidBody& operator=(const RigidBody& p_rigidBody) noexcept;

			//Equality operator
			uint8_t operator==(const RigidBody& p_rigidBody) const noexcept;

			//Value to decrease linear velocity in each iteration of the physics integrate function.
			const float linDamping	{0.995f};
			//Value to decrease angular velocity in each iteration of the physics integrate function.
			const float angDamping	{0.995f};
			//Value that corresponds to the RigidBody's mass.
			float mass		        {1.0f};
			//The inverse value of the mass, it is used many times in the physics engine.
			float inverseMass	    {1/mass};

			//Vector that contains the position of the RigidBody.
			Vector3 position		{0.0f, 0.0f, 0.0f};
			//Holds the linear velocity of the RigidBody.
			Vector3 velocity		{0.0f, 0.0f, 0.0f};
			//Holds the linear acceleration of the RigidBody.
			Vector3 acceleration	{0.0f, 0.0f, 0.0f};
			//Holds the linear acceleration of the RigidBody for the last frame.
			Vector3 lastFrameAccel	{0.0f, 0.0f, 0.0f};
			//Holds the angular orientation of the RigidBody.
			Quaternion orientation  {1.f, 0.f, 0.f, 0.f};
			//Holds the angular velocity of the RigidBody.
			Vector3 rotation        {0.0f, 0.0f, 0.0f};
			//The amount of forces that are applied to the RigidBody.
			Vector3 forceAccum		{0.0f, 0.0f, 0.0f};
			//The amount of torque that are applied to the RigidBody.
			Vector3 torqueAccum		{0.0f, 0.0f, 0.0f};
			//Holds the transformation matrix to convert from local space to world space and vice versa.
			Matrix4 transformMatrix	{1.f};
			//Holds the inverse of the body's inertia tensor. It will be in local space.
			Matrix3 inverseInertiaTensor {1.f};
			//Holds the inverse of the body's inertia tensor but in world space.
			Matrix3 invInertiaTensorWorld {1.f};
			//Holds a variable to know if the body should be updated.
			bool isAwake_	{true};
			//Indicates if the body is allowed to fall asleep.
			bool canSleep_	{true};
	};
}//namespace Ocacho::Physics::RigidBody