/**
 * @file TypeAliases.hpp
 * @author OcachoGames-David (ocachogames@gmail.com)
 * @brief Type aliases for using in the engine
 * @version 0.1
 * @date 2021-12-15
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once 

#include "ogpch.hpp"
#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtx/quaternion.hpp>


namespace Ocacho
{
	//=========================================================================
	//GLM ALIASES
	//=========================================================================

	using Vector2 = glm::vec2;
	using Vector3 = glm::vec3;
	using Vector4 = glm::vec4;
	using Matrix3 = glm::mat3;
	using Matrix4 = glm::mat4;
	using VecMax4 = std::vector<Matrix4>;
	using Quaternion = glm::quat;

	//=========================================================================
	//OWN ALIASES
	//=========================================================================

	//ECS
	using ComponentTypeID = std::size_t;

	//State Machines
	using StateTypeID = std::string;

	//Animation
	using JointID = int16_t;

	//Standard
	template<typename T>
	using UniqPtr = std::unique_ptr<T>;
	template<typename T>
	using RefWrapper = std::reference_wrapper<T>;

	template<typename T, typename T2>
	using UnorderedMap = std::unordered_map<T, T2>;

	template<typename T, typename T2>
	using Map = std::map<T, T2>;

	template<typename... T>
	using Tuple = std::tuple<T...>;

	//Graphic Engine
	using FaceIndex = unsigned int;
	using glID = unsigned int;
	using ShaderID = int;
	using CellID = int32_t;
}


