/**
 * @file CollideFine.hpp
 * @author OcachoGames-Miguel (ocachogames@gmail.com)
 * @brief Contains the detailed collision detection system to generate the contacts.
 * 
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 */

#pragma once

#include <array>

#include "Contacts.hpp"

namespace Ocacho::Physics::RigidBody
{
	//TODO[Otto]:Mirar como hacer para que la variable contactsLeft defina el tamanio del array
	struct CollisionData
	{
		//Indicates the maximum number of contacts per update
		uint32_t contactsLeft_								{50};
		//Holds the last free contact in the contact list array
		uint32_t currentContact_ 							{0};
		//Holds the array of the contacts to be processed in each update
		std::array<Contact, 50> contactList_;
		//Holds the friction value for the contacts generated with this data
		float friction_ 									{.2f};
		//Holds the restitution value for the contacts generated with this data
		float restitution_									{.2f};
		//Holds the tolerance value for the contacts generated with this data
		float tolerance_									{.2f};
	};

	struct CollisionPrimitive
	{
		friend struct CollisionDetector;

		//=====================================================================
		//PUBLIC VARIABLES
		//=====================================================================
		//Holds the body that contains the collision shape
		RigidBody* body_;
		//Holds the offset for the collision shape in relation with the RigidBody which is attached to
		Matrix4 offset_;

		protected:
		//=====================================================================
		//PROTECTED VARIABLES
		//=====================================================================
			//Holds the local transform of the collision shape
			Matrix4 transform_;

			public:
			//=====================================================================
			//PUBLIC METHODS
			//=====================================================================
			/**
			 * @brief Calculates the transform of the collision shape.
			 * 
			 */
			void calculateInternals() noexcept;

			/**
			 * @brief Returns an axis of the Matrix4 offset of the collision shape.
			 * 
			 * @param i Axis to be returned.
			 * @return Vector3 Axis returnes in a Vector3 (w/out scale).
			 */
			Vector3 getAxis(uint8_t i) const noexcept;
	};

	struct CollisionSphere : CollisionPrimitive
	{
		float radius_		{1.f};
	};

	struct CollisionBox : CollisionPrimitive
	{
		Vector3 halfSize_	{1.f, 1.f, 1.f};
	};

	struct CollisionPlane
	{
		Vector3 normal_		{0.f, 1.f, 0.f};
		float offset_		{1.f};

		float distanceToBox(const CollisionBox& p_box) const noexcept;
	};

	struct CollisionDetector
	{
		private:
		//=====================================================================
		//PRIVATE VARIABLES
		//=====================================================================
			//Holds all the possible combinations for each vertex of the box.
			std::array<Vector3, 8> boxMultCombinations { Vector3{1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
														{-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1} };

		public:
		//=====================================================================
		//PUBLIC METHODS
		//=====================================================================
			/**
			 * @brief Method that generates the contacts, if needed, between two spheres.
			 * 
			 * @param p_first The first sphere involved in the collision.
			 * @param p_second The second sphere involved in the collision.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @return uint32_t The number of contacts generatd.
			 */
			uint32_t sphereAndSphere(const CollisionSphere& p_first, const CollisionSphere& p_second
				, CollisionData* p_colData) const noexcept;

			/**
			 * @brief Method that generates the contacts, if needed, between a sphere and a plane
			 * regardless if it is at one side of the plane or the another.
			 * 
			 * @param p_sphere The sphere involved in the collision.
			 * @param p_plane The plane involved in the collision.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @return uint32_t The number of contacts generatd. 
			 */
			uint32_t sphereAndHalfSpace(const CollisionSphere& p_sphere, const CollisionPlane& p_plane
				, CollisionData* p_colData) const noexcept;

			/**
			 * @brief Method that generates the contacts, if needed, between a sphere and a plane.
			 * 
			 * @param p_sphere The sphere involved in the collision.
			 * @param p_plane The plane involved in the collision.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @return uint32_t The number of contacts generatd. 
			 */
			uint32_t sphereAndTruePlane(const CollisionSphere& p_sphere, const CollisionPlane& p_plane
				, CollisionData* p_colData) const noexcept;

			/**
			 * @brief Method that generates the contacts, if needed, between a box and a plane
			 * regardless if it is at one side of the plane or the another.
			 * 
			 * @param p_box The box involved in the collision.
			 * @param p_plane The plane involved in the collision.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @return uint32_t The number of contacts generatd. 
			 */
			uint32_t boxAndHalfSpace(const CollisionBox& p_box, const CollisionPlane& p_plane
				, CollisionData* p_colData) const noexcept;

			/**
			 * @brief Method that generates the contacts, if needed, between a box and a sphere.
			 * 
			 * @param p_box The box involved in the collision.
			 * @param p_sphere The sphere involved in the collision.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @return uint32_t The number of contacts generatd. 
			 */
			uint32_t boxAndSphere(const CollisionBox& p_box, const CollisionSphere& p_sphere
				, CollisionData* p_colData) const noexcept;

			/**
			 * @brief Method that generates the contacts, if needed, between a box and a point.
			 * 
			 * @param p_box The box involved in the collision.
			 * @param p_point The point involved in the collision.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @return uint32_t The number of contacts generatd. 
			 */
			uint32_t boxAndPoint(const CollisionBox& p_box, const Vector3& p_point
				, CollisionData* p_colData) const noexcept;

			/**
			 * @brief Method that generates the contacts, if needed, between two boxes.
			 * 
			 * @param p_first The first box involved in the collision.
			 * @param p_second The second box involved in the collision.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @return uint32_t The number of contacts generatd. 
			 */
			uint32_t boxAndBox(const CollisionBox& p_first, const CollisionBox& p_second
				, CollisionData* p_colData) const noexcept;
		
		private:
		//=====================================================================
		//PRIVATE METHODS
		//=====================================================================
			/**
			 * @brief This method fills the contact data for a box and 
			 * half space collision.
			 * 
			 * @param p_contact Reference to the contact to be filled.
			 * @param p_colData Reference to the collision data to modify its data.
			 * @param p_box Reference to the box.
			 * @param p_plane Reference to the plane.
			 * @param p_vertexDistance Distance between the vertex of the box and
			 * the plane's normal.
			 * @param p_vertexPos Position of the vertex of the box.
			 */
			void fillBoxAndHalfSpaceContact(Contact* p_contact, CollisionData* p_colData
				, const CollisionBox& p_box, const CollisionPlane& p_plane
				, const float p_vertexDistance, const Vector3& p_vertexPos) const noexcept;
			
			/**
			 * @brief This method checks if a box and a sphere are colliding or not.
			 * 
			 * @param p_relCentre The centre of the sphere in the box local coordinates.
			 * @param p_radius Radius of the sphere.
			 * @param p_halfSize The half size of the box.
			 * @return true The box and the sphere are colliding.
			 * @return false The box and the sphere are NOT colliding.
			 */
			bool isBoxAndSphereColliding(const Vector3& p_relCentre
				, const float p_radius, const Vector3& p_halfSize) const noexcept;

			/**
			 * @brief This method fills the contact data for a box and 
			 * a sphere collision.
			 * 
			 * @param p_colData Reference to the collision data to modify its data.
			 * @param p_closestPtWorld The closest point between the box and the sphere.
			 * @param p_centre The centre of the sphere.
			 * @param p_dist Distance between the center of the sphere and the closest
			 * point between the sphere and the box.
			 * @param p_box Reference to the box.
			 * @param p_sphere Reference to the sphere.
			 */
			void fillBoxAndSphereContact(CollisionData* p_colData
				, const Vector3& p_closestPtWorld, const Vector3& p_centre, const float p_dist
				, const CollisionBox& p_box, const CollisionSphere& p_sphere) const noexcept;
			
			/**
			 * @brief This method fills the contact data for a box and 
			 * a point collision.
			 * 
			 * @param p_colData Reference to the collision data to modify its data.
			 * @param p_normal The normal of the contact.
			 * @param p_point Reference to the point.
			 * @param p_minDepth Penetretation of the box into the point.
			 * @param p_box Reference to the box.
			 */
			void fillBoxAndPointContact(CollisionData* p_colData
				, const Vector3& p_normal, const Vector3& p_point, const float p_minDepth
				, const CollisionBox& p_box) const noexcept;

			/**
			 * @brief This method calculates the closest point between a sphere and
			 * a box.
			 * 
			 * @param p_relCentre Relative center of the sphere in box coordinates.
			 * @param p_halfSize Half size of the box.
			 * @return Vector3 Closest point between the sphere and the box.
			 */
			Vector3 clampSphereBoxCoordinates(const Vector3& p_relCentre
				, const Vector3& p_halfSize) const noexcept;
			
			/**
			 * @brief This method calculates the closest distance between a sphere
			 * and a box in a determined axis.
			 * 
			 * @param p_relCentreAxis Axis component of the relative center of
			 * the sphere.
			 * @param p_halfSizeAxis Axis component of the half size of the box.
			 * @return float Closest distance between the sphere and the box
			 * in the given axis values.
			 */
			float clampSphereBoxAxis(const float p_relCentreAxis
				, const float p_halfSizeAxis) const noexcept;

			/**
			 * @brief This method calculates the pentration between two boxes in a
			 * determined axis.
			 * 
			 * @param p_first The first box to make the calculations.
			 * @param p_second The second box to make the calculations.
			 * @param p_axis The axis in which will be the calculations made.
			 * @param p_toCentre Distance between the center of the two boxes.
			 * @return float The calculated penetration.
			 */
			float penetrationOnAxis(const CollisionBox& p_first, const CollisionBox& p_second
				, const Vector3& p_axis, const Vector3& p_toCentre) const noexcept;
			
			/**
			 * @brief The projection of the half size of a box in a given axis.
			 * 
			 * @param p_box Reference to the box.
			 * @param p_axis The axis in which the box will be projected.
			 * @return float The projection of the box in the given axis.
			 */
			float projectionAxisBoxAndBox(const CollisionBox& p_box, const Vector3& p_axis) const noexcept;

			/**
			 * @brief This method checks if there is collision between two boxes in a
			 * determined axis.
			 * 
			 * @param p_first The first box to make the calculations.
			 * @param p_second The second box to make the calculations.
			 * @param p_axis The axis in which will be the calculations made.
			 * @param p_toCentre Distance between the center of the two boxes.
			 * @param p_index The index of the current axis that will be calculated.
			 * @param p_smallestPenetration Reference to the value of the smallest
			 * penetration value that has been calculated.
			 * @param p_smallestCase Reference to the index of the smallest
			 * penetration value that has been calculated.
			 * @return true The boxes are colliding in the given axis.
			 * @return false The boxes are NOT colliding in the given axis.
			 */
			bool tryAxis(const CollisionBox& p_first, const CollisionBox& p_second
				, Vector3 p_axis, const Vector3& p_toCentre, uint32_t p_index
				, float& p_smallestPenetration, uint32_t& p_smallestCase) const noexcept;
			
			void tryIndividualAxisBoxAndBox(const CollisionBox& p_first, const CollisionBox& p_second
				, const Vector3& p_toCentre, float& p_pen, uint32_t& p_best) const noexcept;
			
			void tryPerpendicularAxisBoxAndBox(const CollisionBox& p_first, const CollisionBox& p_second
				, const Vector3& p_toCentre, float& p_pen, uint32_t& p_best) const noexcept;

			/**
			 * @brief This method generates the contact data when we know that
			 * there is collision between two boxes.
			 * 
			 * @param p_first The first box involved in the collision.
			 * @param p_second The second box involved in the collision.
			 * @param p_toCentre Distance between the center of the two boxes.
			 * @param p_colData Pointer to the collision data that will be filled 
			 * with the contact data.
			 * @param p_best The index of the axis in which the contact has occurred.
			 * @param p_pen The value of the penetration of the collision.
			 */
			void fillPointFaceBoxBox(const CollisionBox& p_first, const CollisionBox& p_second
				, const Vector3& p_toCentre, CollisionData* p_colData
				, uint32_t p_best, float p_pen) const noexcept;

			/**
			 * @brief This method calculates the point in which the collision
			 * has occurred between two boxes.
			 * 
			 * @param p_ptOnFirstEdge The point of the edge on the first box
			 * that will be considered for the calculations.
			 * @param p_firstAxis The axis in which the collision has occurred
			 * for the first box.
			 * @param p_firstAxisHalfSize The half size of the axis in which 
			 * the collision has occurred for the first box.
			 * @param p_ptOnSecondEdge he point of the edge on the second box
			 * that will be considered for the calculations.
			 * @param p_secondAxis The axis in which the collision has occurred
			 * for the second box.
			 * @param p_secondAxisHalfSize The half size of the axis in which 
			 * the collision has occurred for the second box.
			 * @param p_useOne Bool to indicate if the contact point is outside
			 * an edge or not, for the first case we use the first's midpoint,
			 * otherwise we use the second's midpoint.
			 * @return Vector3 The point in which the collision has occurred.
			 */
			Vector3 contactPoint(
				const Vector3& p_ptOnFirstEdge, const Vector3& p_firstAxis, float p_firstAxisHalfSize
				, const Vector3& p_ptOnSecondEdge, const Vector3& p_secondAxis, float p_secondAxisHalfSize
				, bool p_useOne) const noexcept;
			
			/**
			 * @brief This method fills the contact data for a collision between
			 * two boxes.
			 * 
			 * @param p_colData Reference to the collision data to modify its data.
			 * @param p_pen Penetration between the two boxes.
			 * @param p_axis Axis in which the boxes are penetrating.
			 * @param p_vertex The contact point of the collision.
			 * @param p_first The first box involved in the collision.
			 * @param p_second The second box involved in the collision.
			 */
			void fillBoxAndBoxContact(CollisionData* p_colData
				, const float p_pen, const Vector3& p_axis, const Vector3& p_vertex
				, const CollisionBox& p_first, const CollisionBox& p_second) const noexcept;
	};

	struct IntersectionTests
	{
		/**
		 * @brief Method that checks if there is collision between a box and a plane.
		 * 
		 * @param p_box The box involved in the collision to check.
		 * @param p_plane The plane involved in the collision to check.
		 * @return true There is collision.
		 * @return false There is NO collision.
		 */
		static bool boxAndHalfSpace(const CollisionBox& p_box, const CollisionPlane& p_plane) noexcept;
	};
}