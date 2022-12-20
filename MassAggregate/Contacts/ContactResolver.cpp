#include "ContactResolver.hpp"

namespace Ocacho::Physics::MassAggregate
{
	ContactResolver::ContactResolver(unsigned p_iterations) noexcept
		: iterations_{p_iterations}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	void
	ContactResolver::setIterations(unsigned p_numIterations) noexcept
	{
		iterations_ = p_numIterations;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	ContactResolver::updateInterpenetrations(std::vector<Contact>& p_contactList, unsigned p_maxIndex) const noexcept
	{
		for(auto& c : p_contactList)
		{
			Vector3 move = p_contactList[p_maxIndex].getMovementForParticle(*c.particles_[0]);

			c.penetration_ -= glm::dot(c.contactNormal_, move);

			if(c.particles_[1])
			{
				move = p_contactList[p_maxIndex].getMovementForParticle(*c.particles_[1]);

				c.penetration_ += glm::dot(c.contactNormal_, move);
			}
		}
	}
	
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	void
	ContactResolver::resolveContacts(std::vector<Contact>& p_contactList, const float p_deltaTime) noexcept
	{
		usedIterations_ = 0;
		auto cListSize = p_contactList.size();

		while (usedIterations_ < iterations_)
		{
			//Find the contact with the largest closing velocity.
			float maxContact = 0.0f;
			unsigned maxIndex = cListSize;

			for(unsigned i=0; i<cListSize; ++i)
			{
				if(p_contactList[i].particles_[0] && p_contactList[i].particles_[1])
				{
					float sepVel = p_contactList[i].calculateSeparatingVelocity();

					if(sepVel < maxContact)
					{
						maxIndex = i;
						maxContact = sepVel;
					}
				}
			}

			if(maxContact >= 0) return;

			p_contactList[maxIndex].resolve(p_deltaTime);

			updateInterpenetrations(p_contactList, maxIndex);

			++usedIterations_;
		}
		
	}
}//namespace Ocacho::Physics::MassAggregate