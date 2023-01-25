namespace Ocacho::Physics::MassAggregate
{
	//=========================================================================
	//CABLE HARD CONSTRAINT
	//=========================================================================

	Cable::Cable(float p_maxLength, float p_restitution = 0.0f) noexcept
		:maxLength_{p_maxLength}, restitution_{p_restitution}
	{}

	//-------------------------------------------------------------------------

	unsigned
	Cable::addContact(std::vector<Contact>& p_contactList) const noexcept
	{
		Contact auxContact = Contact();

		float length = currentLength();

		if(length < maxLength_) return 0;

		auxContact.particles_[0] = particles_[0];
		auxContact.particles_[1] = particles_[1];

		Vector3 contactNormal = particles_[1]->position - particles_[0]->position;

		auxContact.contactNormal_ = glm::normalize(contactNormal);

		auxContact.restitution_ = restitution_;
		auxContact.penetration_ = length - maxLength_;

		p_contactList.push_back(auxContact);

		return 1;
	}

	//=========================================================================
	//ROD HARD CONSTRAINT
	//=========================================================================

	Rod::Rod(const float p_length) noexcept
		:length_{p_length}
	{}
	
	//-------------------------------------------------------------------------

	void
	Rod::evaluateContactNormal(Contact& p_contact, float p_currentLen) const noexcept
	{
		Vector3 contactNormal = particles_[1]->position - particles_[0]->position;

		if(p_currentLen > length_)
		{
			p_contact.contactNormal_ = glm::normalize(contactNormal);
			p_contact.penetration_ = p_currentLen - length_;
		}
		else
		{
			p_contact.contactNormal_ = glm::normalize(contactNormal) * -1.0f;
			p_contact.penetration_ = length_ - p_currentLen;
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	unsigned
	Rod::addContact(std::vector<Contact>& p_contactList) const noexcept
	{
		Contact auxContact = Contact();

		float currentLen = currentLength();

		if(currentLen == length_) return 0;

		auxContact.particles_[0] = particles_[0];
		auxContact.particles_[1] = particles_[1];

		//The contact normal will depend on whether if the rod is extending or compressing
		evaluateContactNormal(auxContact, currentLen);

		//Zero restitution to get no bounciness
		auxContact.restitution_ = 0;

		p_contactList.push_back(auxContact);

		return 1;
	}
}//namespace Ocacho::Physics::MassAggregate