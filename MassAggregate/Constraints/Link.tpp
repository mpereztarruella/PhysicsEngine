namespace Ocacho::Physics::MassAggregate
{
	template<typename CRTPType>
	Link<CRTPType>::Link() {}

	template<typename CRTPType>
	float
	Link<CRTPType>::currentLength() const noexcept
	{
		float distance = glm::distance(particles_[0]->position, particles_[1]->position);

		return distance;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename CRTPType>
	unsigned
	Link<CRTPType>::addContact(std::vector<Contact>& p_contactList) const noexcept
	{
		return static_cast<CRTPType&>(*this).addContact(p_contactList);
	}
}//namespace Ocacho::Physics::MassAggregate