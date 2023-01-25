namespace Ocacho::Physics::RigidBody
{
	template <typename CONT_T>
	void
	ContactResolver::resolveContacts(CONT_T& p_contacts, const uint32_t p_numContacts
		, const float p_deltaTime) noexcept
	{
		if(p_numContacts == 0) return;
		if(!isValid()) return;

		prepareContacts(p_contacts, p_numContacts, p_deltaTime);

		adjustPositions(p_contacts, p_numContacts, p_deltaTime);

		adjustVelocities(p_contacts, p_numContacts, p_deltaTime);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename CONT_T>
	void
	ContactResolver::prepareContacts(CONT_T& p_contacts, const uint32_t p_numContacts
		, const float p_deltaTime) noexcept
	{
		for(uint32_t i = 0; i < p_numContacts; ++i)
		{
			p_contacts[i].calculateInternals(p_deltaTime);
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename CONT_T>
	void
	ContactResolver::adjustVelocities(CONT_T& p_contacts, const uint32_t p_numContacts
		, const float p_deltaTime) noexcept
	{
		Vector3 velocityChange[2], rotationChange[2];

		velocityIterationsUsed_ = 0;

		while(velocityIterationsUsed_ < velocityIterations_)
		{
			float max {velocityEpsilon_};
			uint32_t index = p_numContacts;

			//First find the contact with the maximum magnitude of probable velocity change
			for(uint32_t i = 0; i < p_numContacts; ++i)
			{
				if(p_contacts[i].desiredDeltaVelocity_ > max)
				{
					max = p_contacts[i].desiredDeltaVelocity_;
					index = i;
				}
			}

			if(index == p_numContacts) break;

			//Match the awake state for the bodies in the contact
			p_contacts[index].matchAwakeState();

			//Do the resolution on the contact that came out top
			p_contacts[index].applyVelocityChange(velocityChange, rotationChange);

			recomputeClosingVelocities(p_contacts, p_numContacts
				, velocityChange, rotationChange
				, index , p_deltaTime);

			++velocityIterationsUsed_;
		}
	}

	template <typename CONT_T>
	void
	ContactResolver::recomputeClosingVelocities(CONT_T& p_contacts, const uint32_t p_numContacts
		, Vector3 p_velocityChange[2], Vector3 p_rotationChange[2]
		, const uint32_t p_index, const float p_deltaTime) noexcept
	{
		Vector3 deltaVel;

		for(uint32_t i = 0; i < p_numContacts; ++i)
		{
			for(uint8_t b = 0; b < 2; ++b)
			{
				if(p_contacts[i].bodies_[b])
				{
					for(uint8_t d = 0; d < 2; ++d)
					{
						if(p_contacts[i].bodies_[b] == p_contacts[p_index].bodies_[d])
						{
							deltaVel = glm::cross(p_rotationChange[d], p_contacts[i].relativeContactPosition_[b]);
							deltaVel = p_velocityChange[d] + deltaVel;

							//The sign of the change is negative if we're dealing with
							//the second body of the contact
							p_contacts[i].contactVelocity_ += glm::inverse(p_contacts[i].contactToWorld_)
								* deltaVel * (b?-1.f:1.f);
							
							p_contacts[i].calculateDesiredDeltaVelocity(p_deltaTime);
						}
					}
				}
			}
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename CONT_T>
	void
	ContactResolver::adjustPositions(CONT_T& p_contacts, const uint32_t p_numContacts
		, const float p_deltaTime) noexcept
	{
		Vector3 linearChange[2], angularChange[2];

		positionIterationsUsed_ = 0;

		while(positionIterationsUsed_ < positionIterations_)
		{
			float max {positionEpsilon_};
			uint32_t index = p_numContacts;

			//First find the contact with the maximum magnitude of probable position change
			for(uint32_t i = 0; i < p_numContacts; ++i)
			{
				if(p_contacts[i].penetration_ > max)
				{
					max = p_contacts[i].penetration_;
					index = i;
				}
			}

			if(index == p_numContacts) break;

			//Match the awake state for the bodies in the contact
			p_contacts[index].matchAwakeState();

			//Do the resolution on the contact that came out top
			p_contacts[index].applyPositionChange(linearChange, angularChange,
				max);

			recomputePenetrations(p_contacts, p_numContacts
				, linearChange, angularChange
				, index, p_deltaTime);

			++positionIterationsUsed_;
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename CONT_T>
	void
	ContactResolver::recomputePenetrations(CONT_T& p_contacts, const uint32_t p_numContacts
		, Vector3 p_linearChange[2], Vector3 p_angularChange[2]
		, const uint32_t p_index, const float p_deltaTime) noexcept
	{
		Vector3 deltaPosition;

		for(uint32_t i = 0; i < p_numContacts; ++i)
		{
			for(uint8_t b = 0; b < 2; ++b)
			{
				if(p_contacts[i].bodies_[b])
				{
					for(uint8_t d = 0; d < 2; ++d)
					{
						if(p_contacts[i].bodies_[b] == p_contacts[p_index].bodies_[d])
						{
							deltaPosition = glm::cross(p_angularChange[d], p_contacts[i].relativeContactPosition_[b]);
							deltaPosition = p_linearChange[d] + deltaPosition;

							//The sign of the change is negative if we're dealing with
							//the second body of the contact
							p_contacts[i].penetration_ += glm::dot(deltaPosition, p_contacts[i].contactNormal_)
								* (b?1.f:-1.f);
						}
					}
				}
			}
		}
	}
}