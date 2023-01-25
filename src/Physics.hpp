/**
 * @file Physics.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief This file would contain the main functions for the physics engine.
 * 
 * 
 * @version 0.1
 * @date 2022-01-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "Utility/TypeAliases.hpp"

#include "MassAggregate/Particle.hpp"
#include "RigidBody/RigidBody.hpp"

namespace Ocacho::Physics
{
	/**
	 * @brief Transform a given point from local space to world space.
	 * 
	 * @param p_transformMat The transform matrix that will be used to make the conversion.
	 * @param p_point The point in local space coordinates.
	 * @return Vector3 The point in world space coordinates.
	 */
	Vector3 TransformPointToWorld(const Matrix4& p_transformMat, const Vector3& p_point) noexcept;

	/**
	 * @brief Transform a given direction from local space to world space.
	 * 
	 * @param p_transformMat The transform matrix that will be used to make the conversion.
	 * @param p_dir The direction in local space coordinates.
	 * @return Vector3 The direction in world space coordinates.
	 */
	Vector3 TransformDirToWorld(const Matrix4& p_transformMat, const Vector3& p_dir) noexcept;
}//namespace Ocacho::Physics

namespace Ocacho::Physics::MassAggregate
{
	/**
	 * @brief Function that clears all the accumulated forces for a Particle.
	 * 
	 * @param p_forceAccum reference Vector3 of the entity's accumulated forces.
	 */
	void ClearForcesParticle(Vector3& p_forceAccum) noexcept;

	/**
	 * @brief Integrator function of the mass-aggregate engine: updates the velocity and the position
	 * passed by parameter based on the acceleration and the amount of forces.
	 * 
	 * @param p_particle Reference to the particle to integrate
	 * @param p_deltaTime Elapsed time between frames.
	 */
	void IntegrateParticle(Particle& p_particle, const float p_deltaTime) noexcept;
}//namespace Ocacho::Physics::MassAggregate

namespace Ocacho::Physics::RigidBody
{
	/**
	 * @brief Function that clears all the accumulated forces and torques for a RigidBody.
	 * 
	 * @param p_forceAccum Reference Vector3 of the RigidBody's accumulated forces.
	 * @param p_torqueAccum Reference Vector3 of the RigidBody's accumulated torques.
	 */
	void ClearAccumulator(Vector3& p_forceAccum, Vector3& p_torqueAccum) noexcept;

	/**
	 * @brief Integrator function of the rigidbody engine: updates the velocities, position and
	 * orientation for a given RigidBody.
	 * 
	 * @param p_rigidBody Reference to the RigidBody to integrate.
	 * @param p_deltaTime Elapsed time between frames.
	 */
	void IntegrateRigidBody(RigidBody& p_rigidBody, const float p_deltaTime) noexcept;

	/**
	 * @brief Calculates the transformation matrix for a RigidBody.
	 * 
	 * @param p_transformMatrix Reference Matrix4 of the RigidBody's transform matrix.
	 * @param p_position Constant reference to the RigidBody's position.
	 * @param p_orientation Constant reference to the RigidBody's orientation.
	 */
	void CalculateTransformMatrix(Matrix4& p_transformMatrix
		, const Vector3& p_position
		, const Quaternion& p_orientation) noexcept;

	/**
	 * @brief Sets the inverse inertia tensor given an inertia tensor.
	 * 
	 * @param p_inertiaTensor Inertia tensor to set the values.
	 * @param p_inverseInertiaTensor Inverse inertia tensor that will be modified.
	 */
	void SetInertiaTensor(Matrix3 p_inertiaTensor
		, Matrix3& p_inverseInertiaTensor) noexcept;
	
	/**
	 * @brief Changes the space coordinates of the inertia tensor from local to world space.
	 * 
	 * @param p_invInertTensWorld Reference to the inertia tensor that will be set in world space.
	 * @param p_invInertTensLocal Constant reference to the inertia tensor in local space.
	 * @param p_orientation Constant reference of the orientation of the RigidBody.
	 * @param p_transfMat Constant reference of the transform matrix of the RigidBody.
	 */
	void TransformInertiaTensor(Matrix3& p_invInertTensWorld
		, const Matrix3& p_invInertTensLocal
		, const Matrix4& p_transfMat) noexcept;
	
	/**
	 * @brief Calls the methods that updates the derived data for the Transform Matrix
	 * and the inverse inertia tensor.
	 * 
	 * @param p_rigidBody The RigidBody that will get it's data updated.
	 */
	void CalculateDerivedData(RigidBody& p_rigidBody) noexcept;
	
	/**
	 * @brief Adds a force given to a RigidBody.
	 * 
	 * @param p_force Constant reference of the force to apply.
	 * @param p_rigidBody Reference of the RigidBody that will have the force applied.
	 */
	void AddForceToRigidBody(const Vector3& p_force, RigidBody& p_rigidBody) noexcept;

	/**
	 * @brief Adds a torque given to a RigidBody.
	 * 
	 * @param p_torque Constant reference of the torque to apply.
	 * @param p_rigidBody Reference of the RigidBody that will have the torque applied.
	 */
	void AddTorqueToRigidBody(const Vector3& p_torque, RigidBody& p_rigidBody) noexcept;

	/**
	 * @brief Adds a force to a point of a RigidBody. Point in local space.
	 * 
	 * @param p_force Force to be applied.
	 * @param p_point The point of the body where the force will be applied. In Local space.
	 * @param p_rigidBody RigidBody to which is being applied the force.
	 */
	void AddForceAtLocalPoint(const Vector3& p_force
		, const Vector3& p_point
		, RigidBody& p_rigidBody) noexcept;

	/**
	 * @brief Adds a force to a point of a RigidBody. Point in world space.
	 * 
	 * @param p_force Force to be applied.
	 * @param p_point The point of the body where the force will be applied. In World space.
	 * @param p_rigidBody RigidBody to which is being applied the force.
	 */
	void AddForceAtWorldPoint(const Vector3& p_force
		, const Vector3& p_point
		, RigidBody& p_rigidBody) noexcept;
}//namespace Ocacho::Physics::RigidBody