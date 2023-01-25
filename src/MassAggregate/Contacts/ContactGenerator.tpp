namespace Ocacho::Physics::MassAggregate
{   
	template<typename CRTPType>
	unsigned
	ContactGenerator<CRTPType>::addContact(std::vector<Contact>& p_contactList) const noexcept
	{
		return static_cast<CRTPType&>(*this).addContact(p_contactList);
	}

}//namespace Ocacho::Physics::MassAggregate