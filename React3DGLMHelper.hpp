/**
 * @file React3DGLMHelper.hpp
 * @author OcachoGames-Antonio (ocachogames@gmail.com)
 * @brief Function helpers for transforming Rect3D types into Ocacho types
 * @version 0.1
 * @date 2022-03-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "Utility/TypeAliases.hpp"
#include "Maths/Transform.hpp"
#include <reactphysics3d/reactphysics3d.h>

namespace Ocacho::Reactd3DGLMHelper
{
	/**
	 * @brief Converts a react3D Transform to a GLM Transform.
	 * 
	 * @param p_position React3D position.
	 * @param p_orientation React3D orientation.
	 * @return Transform Transform with GLM types.
	 */
	inline Transform
	toGLMTransform(const reactphysics3d::Vector3& p_position, 
				const reactphysics3d::Quaternion& p_orientation)
	{
		return
		{
			{p_position.x,p_position.y,p_position.z},
			{p_orientation.w,p_orientation.x,p_orientation.y,p_orientation.z},
			{1.f, 1.f, 1.f}
		};
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	/**
	 * @brief Converts a react3D Vector3 to a GLM Vector3.
	 * 
	 * @param p_vector React3D Vector3.
	 * @return Vector3 GLM Vector3.
	 */
	inline Vector3
	toGLMVector3(const reactphysics3d::Vector3& p_vector)
	{
		return {p_vector.x,p_vector.y,p_vector.z};
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	/**
	 * @brief Converts a react3D Quaternion to a GLM Quaternion.
	 * 
	 * @param p_quaternion React3D Quaternion.
	 * @return Quaternion GLM Quaternion.
	 */
	inline Quaternion
	toGLMQuaternion(const reactphysics3d::Quaternion& p_quaternion)
	{
		return {p_quaternion.w, p_quaternion.x, p_quaternion.y, p_quaternion.z};
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	inline reactphysics3d::Quaternion
	toReact3DQuaternion(const Ocacho::Quaternion& p_quaternion)
	{
		return {p_quaternion.x, p_quaternion.y, p_quaternion.z, p_quaternion.w};
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	/**
	 * @brief Converts a GLM Transform to a react3D Transform.
	 * 
	 * @param p_trns GLM Transform.
	 * @return reactphysics3d::Transform React3D Transform.
	 */
	inline reactphysics3d::Transform
	toR3DTransform(const Transform& p_trns)
	{ 
		return
		{
			{p_trns.position.x,p_trns.position.y,p_trns.position.z},
			{p_trns.rotation.x,p_trns.rotation.y,p_trns.rotation.z,p_trns.rotation.w},
		};
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	/**
	 * @brief Converts a GLM Vector3 to a react3D Vector3.
	 * 
	 * @param p_vec GLM Vector3.
	 * @return reactphysics3d::Vector3 React3D Vector3.
	 */
	inline reactphysics3d::Vector3
	toR3DVector(const Ocacho::Vector3& p_vec)
	{ 
		return {p_vec.x,p_vec.y,p_vec.z};
	}
} // namespace Ocacho::Rectd3DGLMHelper