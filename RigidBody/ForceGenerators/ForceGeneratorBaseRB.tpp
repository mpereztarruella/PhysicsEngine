namespace Ocacho::Physics::RigidBody
{
	template<typename CRTPType>
	void
	ForceGeneratorRB<CRTPType>::updateForce(RigidBody& p_rigidBody, const float p_deltaTime) noexcept
	{
		static_cast<CRTPType&>(*this).updateForce(p_rigidBody, p_deltaTime);
	}

	//=========================================================================
	//FORCE REGISTRATION
	//=========================================================================
	// TODO [Otto]: Por que el auto p_forceGenerator no me crea una copia?
	template<typename... genTypes>
	ForceRegistrationRB<genTypes...>::ForceRegistrationRB(RigidBody& p_rigidBody, auto p_forceGenerator) noexcept
		:rigidBody_{p_rigidBody}, forceGen_{p_forceGenerator}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	ForceRegistrationRB<genTypes...>& 
	ForceRegistrationRB<genTypes...>::operator=(const ForceRegistrationRB& p_fReg) noexcept
	{
		if(this == &p_fReg)
			return *this;

		rigidBody_ = p_fReg.rigidBody_;
		forceGen_ = p_fReg.forceGen_;

		return *this;
	}

	//=========================================================================
	//FORCE REGISTRY
	//=========================================================================

	template<typename... genTypes>
	void
	ForceRegistryRB<genTypes...>::add(RigidBody& p_rigidBody, auto& p_fGen) noexcept
	{
		registrations_.emplace_back(ForceRegistrationRB<genTypes...>{p_rigidBody, &p_fGen});
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	template<typename generatorType>
	void
	ForceRegistryRB<genTypes...>::remove(RigidBody& p_rigidBody) noexcept
	{
		for(auto i = 0; i < registrations_.size(); ++i)
		{
			if(&registrations_[i].rigidBody_ == &p_rigidBody)
			{
				auto* generator = std::get_if<generatorType*>(&registrations_[i].forceGen_);

				if(generator)
				{
					registrations_.erase(registrations_.begin() + i);
					return;
				}
			}
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	void
	ForceRegistryRB<genTypes...>::clear() noexcept
	{
		registrations_.clear();
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	void
	ForceRegistryRB<genTypes...>::updateForces(const float p_deltaTime) noexcept
	{
		for(const auto& r : registrations_)
		{
			RigidBody& p = r.rigidBody_;
			auto& force = r.forceGen_;

			std::visit([&](auto& force)
			{
				force->updateForce(p, p_deltaTime);
			}, force);
		}
	}
}//namespace Ocacho::Physics::RigidBody