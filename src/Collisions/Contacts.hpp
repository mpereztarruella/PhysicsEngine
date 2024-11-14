/**
 * @file Contacts.hpp
 * @author Miguel Perez Tarruella (mpereztarruella@gmail.com)
 * @brief Contains the structure of data and the different methods to create the contacts
 * for the collision system.
 * 
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 */

#pragma once

#include "Utility/TypeAliases.hpp"

#include "RigidBody/RigidBody.hpp"

namespace Ocacho::Physics::RigidBody
{

	//=========================================================================
	//CONTACT
	//=========================================================================
	struct Contact
	{
		friend class ContactResolver;

		//Holds the two bodies involved in the contact
		RigidBody* bodies_[2];
		//Holds the friction coefficient for the contact
		float friction_				{.5f};
		//Holds the restitution coefficient for the contact
		float restitution_			{.5f};
		//Holds the position of the contact in world space
		Vector3 contactPoint_		{0, 0, 0};
		//Holds the direction of the contact in world space
		Vector3 contactNormal_		{0, 0, 0};
		//Holds the amount of interpenetration at the contact point
		float penetration_			{0};

		/**
		 * @brief Method to set the body, friction and restitution data of the contact.
		 * 
		 * @param p_first RigidBody pointer to be set to the first index of bodies_.
		 * @param p_second RigidBody pointer to be set to the second index of bodies_.
		 * @param p_friction Friction coefficient to set to the contact.
		 * @param p_restitution Restitution coefficient to set to the contact.
		 */
		void setBodyData(RigidBody* p_first, RigidBody* p_second
			, float p_friction, float p_restitution)
		{
			bodies_[0] = p_first;
			bodies_[1] = p_second;
			friction_ = p_friction;
			restitution_ = p_restitution;
		}

		protected:
			//=================================================================
			//PROTECTED VARIABLES
			//=================================================================
			//Matrix that converts local coordinates of the contact to World coordinates
			Matrix3 contactToWorld_				{1.f};
			//Holds the closing velocity for this contact
			Vector3 contactVelocity_			{0.f};
			//Holds the required change in velocity for this contact to be resolved
			float desiredDeltaVelocity_			{0.f};
			//Holds the world space position of the contact relative to the center
			// of each body
			Vector3 relativeContactPosition_[2] {};
			//Hold the limit under which the restitution will be zero
			constexpr static float velocityLimit_	{.25f};

		protected:
			//=================================================================
			//PROTECTED METHODS
			//=================================================================
			/**
			 * @brief Calculates the internal data needed for the contact
			 * resolution.
			 * 
			 * @param p_deltaTime Ellapsed time between frames.
			 */
			void calculateInternals(const float p_deltaTime) noexcept;

			/**
			 * @brief This method swaps the order of the bodies involved in the contact.
			 * 
			 */
			void swapBodies() noexcept;

			/**
			 * @brief This method checks if some of the bodies involved in the contact
			 * is awake, if so set the other body awake too.
			 * 
			 */
			void matchAwakeState() noexcept;

			/**
			 * @brief Calculates the desired value for the delta velocity.
			 * 
			 * @param p_deltaTime Elapsed time between frames.
			 */
			void calculateDesiredDeltaVelocity(const float p_deltaTime) noexcept;

			/**
			 * @brief This method calculates the velocity of the contact point on
			 * the given body.
			 * 
			 * @param p_index The index of the body in the array of bodies.
			 * @param p_deltaTime Elapsed time between frames.
			 * @return Vector3 The calculated velocity.
			 */
			Vector3 calculateLocalVelocity(uint8_t p_index, const float p_deltaTime) noexcept;

			/**
			 * @brief Calculates an orthonormal basis for the contact point to
			 * create a local coordinate space.
			 * 
			 */
			void calculateContactBasis() noexcept;
			
			void applyImpulse(const Vector3& p_impulse, RigidBody* p_body
				, Vector3& p_velChange, Vector3 p_rotChange) noexcept;

			/**
			 * @brief This methods performs and inertia-weighted based resolution
			 * for this contact.
			 * 
			 * @param p_velChange Array of velocity change for the bodies.
			 * @param p_rotChange Array of rotation change for the bodies.
			 */
			void applyVelocityChange(Vector3 p_velChange[2], Vector3 p_rotChange[2]) noexcept;

			/**
			 * @brief This method performs the resolution of the velocity change
			 * for a single body.
			 * 
			 * @param p_index Index of the body in the bodies array.
			 * @param p_impulse Reference to the calculated impulse.
			 * @param p_invInerTensor Inverse inertia tensor corresponding to the 
			 * index of the body.
			 * @param p_velChange Reference to the velocity change vector of the body.
			 * @param p_rotChange Reference to the rotation change vector of the body.
			 */
			void applyVelocityChangeToBody(uint8_t p_index, const Vector3& p_impulse
				, const Matrix3 p_invInerTensor, Vector3& p_velChange, Vector3& p_rotChange) noexcept;

			/**
			 * @brief This methods performs and inertia-weighted penetration resolution
			 * for this contact.
			 * 
			 * @param p_linChange Array for the linear change of the bodies involved
			 * in the contact.
			 * @param p_angChange Array for the angular change of the bodies involved
			 * in the contact.
			 * @param p_penetration The penetration depth.
			 */
			void applyPositionChange(Vector3 p_linChange[2], Vector3 p_angChange[2]
				, const float p_penetration) noexcept;
			
			/**
			 * @brief This method performs the resolution of the velocity change
			 * for a single body.
			 * 
			 * @param p_index Index of the body in the bodies array.
			 * @param p_angInert The angular inertia for the body.
			 * @param p_linInert The linear inertia for the body.
			 * @param p_totalInert The total amount of inertia.
			 * @param p_linChange Reference to the linear change of the body.
			 * @param p_angChange Reference to the angular change of the body.
			 */
			void applyPositionChangeToBody(uint8_t p_index
				, const float p_angInert, const float p_linInert, const float p_totalInert
				, Vector3& p_linChange, Vector3& p_angChange, const float p_penetration) noexcept;
			
			/**
			 * @brief This method limits the angular move.
			 * 
			 * @param p_maxMagnitude The maximum magnitude for the angular move.
			 * @param p_angMove The current angular move.
			 * @param p_linMove The current linear move.
			 */
			void limitMoveToPositionChange(const float p_maxMagnitude
				, float& p_angMove, float& p_linMove) const noexcept;

			/**
			 * @brief This method calculates the impulse needed to resolve this 
			 * contact, given that the contact has no friction.
			 * 
			 * @param p_invInertiaTensor The inverse inertia tensors of the
			 * bodies.
			 * @return Vector3 The calculated impulse.
			 */
			Vector3 calculateFrictionlessImpulse(Matrix3 p_invInertiaTensor[2]) noexcept;

			/**
			 * @brief This method calculates the impulse needed to resolve
			 * this contact for a single body.
			 * 
			 * @param p_index Index of the body in the bodies array.
			 * @param p_invInerTensor nverse inertia tensor corresponding to the 
			 * index of the body.
			 * @return float Calculated delta velocity for the body.
			 */
			float calculateFrictionlessImpulseBody(uint8_t p_index
				, const Matrix3& p_invInerTensor) noexcept;
	};

	//=========================================================================
	//CONTACT RESOLVER
	//=========================================================================

	class ContactResolver
	{
		protected:
			//=================================================================
			//PROTECTED VARIABLES
			//=================================================================
			//Holds the number of iterations to perform the velocity resolution
			uint32_t velocityIterations_	{50};
			//Holds the number of iterations to perform the position resolution
			uint32_t positionIterations_	{50};
			//The epsilon value for the velocity to avoid instability.
			float velocityEpsilon_			{0.01f};
			//The epsilon value for the position to avoid instability.
			float positionEpsilon_			{0.01f};
		
		public:
			//=================================================================
			//PUBLIC VARIABLES
			//=================================================================
			uint32_t velocityIterationsUsed_;
			uint32_t positionIterationsUsed_;
		
		private:
			//=================================================================
			//PRIVATE VARIABLES
			//=================================================================
			bool validSettings_;
		
		public:
			//=================================================================
			//PUBLIC METHODS
			//=================================================================
			ContactResolver(uint32_t p_iterations
				, float velocityEpsilon=0.01f, float positionEpsilon=0.01f) noexcept;
			
			ContactResolver(uint32_t p_velocityIterations, uint32_t p_positionIterations
				, float velocityEpsilon=0.01f, float positionEpsilon=0.01f) noexcept;
			
			/**
			 * @brief Check if the values of the variables are valid.
			 * 
			 * @return true The values are valid.
			 * @return false The values are NOT valid.
			 */
			bool isValid() const noexcept;

			/**
			 * @brief Set the iterations for the velocity and the position resolution.
			 * 
			 * @param p_velocityIterations The number of the iterations for the velocity resolution.
			 * @param p_positionIterations The number of the iterations for the position resolution.
			 */
			void setIterations(const uint32_t p_velocityIterations, const uint32_t p_positionIterations) noexcept;

			/**
			 * @brief Set the epsilon for the velocity and the position resolution.
			 * 
			 * @param p_velocityEpsilon The epsilon for the velocity resolution.
			 * @param p_positionEpsilon The epsilon for the position resolution.
			 */
			void setEpsilon(const float p_velocityEpsilon, const float p_positionEpsilon) noexcept;

			/**
			 * @brief This method calls the needed methods to make the contact resolution.
			 * 
			 * @tparam CONT_T The container type for the object that contains the contacts.
			 * @param p_contacts The object that contains the contacts to resolve.
			 * @param p_deltaTime Ellapsed time between frames.
			 */
			template <typename CONT_T>
			void resolveContacts(CONT_T& p_contacts, const uint32_t p_numContacts
				, const float p_deltaTime) noexcept;
		protected:
			//=================================================================
			//PROTECTED METHODS
			//=================================================================
			/**
			 * @brief 
			 * 
			 * @tparam CONT_T The container type for the object that contains the contacts.
			 * @param p_contacts The object that contains the contacts to resolve.
			 * @param p_deltaTime Ellapsed time between frames. 
			 */
			template <typename CONT_T>
			void prepareContacts(CONT_T& p_contacts, const uint32_t p_numContacts
				, const float p_deltaTime) noexcept;

			/**
			 * @brief This method adjusts the velocity for the bodies involved in any
			 * type of contact.
			 * 
			 * @tparam CONT_T The container type for the object that contains the contacts.
			 * @param p_contacts The object that contains the contacts to resolve.
			 * @param p_deltaTime Ellapsed time between frames.
			 */
			template <typename CONT_T>
			void adjustVelocities(CONT_T& p_contacts, const uint32_t p_numContacts
				, const float p_deltaTime) noexcept;

			/**
			 * @brief This method recalculates the closing velocities for the contacts
			 * after have applied a velocity change to a single contact.
			 * 
			 * @tparam CONT_T The container type for the object that contains the contacts.
			 * @param p_contacts The object that contains the contacts to resolve.
			 * @param p_index 
			 * @param p_deltaTime Ellapsed time between frames.
			 */
			template <typename CONT_T>
			void recomputeClosingVelocities(CONT_T& p_contacts, const uint32_t p_numContacts
				, Vector3 p_velocityChange[2], Vector3 p_rotationChange[2]
				, const uint32_t p_index, const float p_deltaTime) noexcept;

			/**
			 * @brief This method adjusts the position for the bodies involved in any
			 * type of contact.
			 * 
			 * @tparam CONT_T The container type for the object that contains the contacts.
			 * @param p_contacts The object that contains the contacts to resolve.
			 * @param p_deltaTime Ellapsed time between frames. 
			 */
			template <typename CONT_T>
			void adjustPositions(CONT_T& p_contacts, const uint32_t p_numContacts
				, const float p_deltaTime) noexcept;

			/**
			 * @brief This method recalculates the penetration for the contacts
			 * after have applied a velocity change to a single contact.
			 * 
			 * @tparam CONT_T The container type for the object that contains the contacts.
			 * @param p_contacts The object that contains the contacts to resolve.
			 * @param p_index 
			 * @param p_deltaTime Ellapsed time between frames.
			 */
			template <typename CONT_T>
			void recomputePenetrations(CONT_T& p_contacts, const uint32_t p_numContacts
				, Vector3 p_linearChange[2], Vector3 p_angularChange[2]
				, const uint32_t p_index, const float p_deltaTime) noexcept;
	};
}//namespace Ocacho::Physics::RigidBody

#include "Contacts.tpp"