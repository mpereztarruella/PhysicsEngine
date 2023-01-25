/**
 * @file BoundingVolumes.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief Contains the structures for the different bounding volumes.
 * 
 * @version 0.1
 * @date 2022-03-08
 * 
 * @copyright Copyright (c) 2022
 */

#pragma once

#include <numbers>

#include "Utility/TypeAliases.hpp"

namespace Ocacho::Physics::Collider
{
	constexpr auto PI {std::numbers::pi};

	//=========================================================================
	//BOUNDING SPHERE
	//=========================================================================
	struct BoundingSphere
	{
		BoundingSphere(const Vector3& p_centre = {0, 0, 0}, float p_radius = 1.f) noexcept;

		/**
		 * @brief Construct a new Bounding Sphere object with two bounding spheres given.
		 * 
		 * @param p_first One of the bounding spheres.
		 * @param p_second The other bounding sphere.
		 */
		BoundingSphere(const BoundingSphere& p_first, const BoundingSphere& p_second) noexcept;

		/**
		 * @brief Checks if the bounding sphere overlaps with other bounding sphere.
		 * 
		 * @param p_other The other bounding sphere to check the overlapping.
		 * @return true The spheres are overlapping.
		 * @return false The spheres are NOT overlapping.
		 */
		bool overlaps(const BoundingSphere& p_other) const noexcept;

		/**
		 * @brief Calculates how much this bounding sphere would have to grow
		 * to incorporate the given bounding sphere.
		 * 
		 * @param p_other The bounding sphere to incorporate.
		 * @return float The growth of the bounding sphere.
		 */
		float getGrowth(const BoundingSphere& p_other) const noexcept;

		/**
		 * @brief Get the Size object.
		 * 
		 * @return float The size of the object.
		 */
		float getVolume() const noexcept;

		private:

		Vector3 centre_ {0, 0, 0};
		float radius_ {1.f};
	};

	//=========================================================================
	//BOUNDING BOX
	//=========================================================================
	struct BoundingBox
	{
		BoundingBox(const Vector3& p_centre = {0, 0, 0}, const Vector3& p_halfExtents = {.5f, .5f, .5f}) noexcept;

		/**
		 * @brief Construct a new Bounding Sphere object with two bounding spheres given.
		 * 
		 * @param p_first One of the bounding spheres.
		 * @param p_second The other bounding sphere.
		 */
		BoundingBox(const BoundingBox& p_first, const BoundingBox& p_second) noexcept;

		/**
		 * @brief Checks if the bounding sphere overlaps with other bounding sphere.
		 * 
		 * @param p_other The other bounding sphere to check the overlapping.
		 * @return true The spheres are overlapping.
		 * @return false The spheres are NOT overlapping.
		 */
		bool overlaps(const BoundingBox& p_other) const noexcept;

		/**
		 * @brief Calculates how much this bounding sphere would have to grow
		 * to incorporate the given bounding sphere.
		 * 
		 * @param p_other The bounding sphere to incorporate.
		 * @return float The growth of the bounding sphere.
		 */
		float getGrowth(const BoundingBox& p_other) const noexcept;

		/**
		 * @brief Get the Size object.
		 * 
		 * @return float The size of the object.
		 */
		float getVolume() const noexcept;

		private:

		Vector3 centre_ {0, 0, 0};
		Vector3 halfExtents_ {1.f, 1.f, 1.f};
	};
}
