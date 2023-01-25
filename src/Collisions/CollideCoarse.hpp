/**
 * @file CollideCoarse.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief Contains the first collision detection system to filter possible collisions.
 * 
 * @version 0.1
 * @date 2022-03-14
 * 
 * @copyright Copyright (c) 2022
 */
#pragma once

#include <array>

#include "RigidBody/RigidBody.hpp"

namespace Ocacho::Physics::RigidBody
{
	/**
	 * @brief Struct to store a potential contact to check later
	 * 
	 */
	struct PotentialContact
	{
		RigidBody* bodies_[2];
	};

	template <typename BoundingVolume>
	struct BVHNode
	{
		//=====================================================================
		//PUBLIC VARIABLES
		//=====================================================================
		using children_t = Ocacho::UniqPtr<BVHNode<BoundingVolume>>;
		std::array<children_t, 2> children_;

		BoundingVolume volume_;

		RigidBody* body_ {nullptr};

		BVHNode* parent_ {nullptr};

		//=====================================================================
		//PUBLIC METHODS
		//=====================================================================
		BVHNode(BVHNode* p_parent, const BoundingVolume& p_volume, RigidBody* p_body = nullptr) noexcept;

		~BVHNode();

		/**
		 * @brief This method indicates if the node is a leaf of the tree hierarchy.
		 * 
		 * @return true if the node is a leaf.
		 * @return false if the node is not a leaf.
		 */
		bool isLeaf() const noexcept;

		/**
		 * @brief This method calls the method to calculate the potential contacts between its children
		 * and returns the number of possible contacts.
		 * 
		 * @param p_contacts Pointer to the possible contact list.
		 * @param p_limit Limit number for the possible contacts.
		 * @return unsigned Number of possible contacts.
		 */
		unsigned getPotentialContacts(PotentialContact* p_contacts, unsigned p_limit) const noexcept;

		/**
		 * @brief Inserts a new node in the tree hierarchy.
		 * 
		 * @param p_newVolume The bounding volume for the new node.
		 * @param p_newBody The rigidbody that will be attached to the new node.
		 */
		void insert(const BoundingVolume& p_newVolume, RigidBody* p_newBody) noexcept;

		/**
		 * @brief This method calls the overlap method between the volumes of the nodes.
		 * 
		 * @param p_other The other node to check the overlapping.
		 * @return true if the nodes are overlapping.
		 * @return false if the nodes are not overlapping.
		 */
		bool overlaps(const BVHNode<BoundingVolume>* p_other) const noexcept;

		protected:
		//=====================================================================
		//PROTECTED METHODS
		//=====================================================================

		/**
		 * @brief Calculates the possible contacts between two nodes.
		 * 
		 * @param p_other The other node to check the contacts.
		 * @param p_contacts Pointer to the contact list.
		 * @param p_limit Limit number for the possible contacts.
		 * @return unsigned Number of possible contacts calculated.
		 */
		unsigned getPotentialContactsWith(const BVHNode<BoundingVolume>* p_other,
			PotentialContact* p_contacts, unsigned p_limit) const noexcept;

		/**
		 * @brief This method recalculates the volume of the bounding volumes in the tree
		 * it will be called when an insert is made.
		 * 
		 * @param p_recurse Indicates if we have to recurse up in the tree or not.
		 */
		void recalculateBoundingVolume(bool p_recurse = true) noexcept;
	};
}//namespace Ocacho::Physics::RigidBody

#include "CollideCoarse.tpp"