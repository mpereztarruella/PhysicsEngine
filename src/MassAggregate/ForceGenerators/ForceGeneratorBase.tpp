namespace Ocacho::Physics::MassAggregate
{
	template<typename CRTPType>
	void
	ForceGenerator<CRTPType>::updateForce(Particle& p_particle, const float p_deltaTime) const noexcept
	{
		static_cast<CRTPType&>(*this).updateForce(p_particle, p_deltaTime);
	}

	//=========================================================================
	//FORCE REGISTRATION
	//=========================================================================
	template<typename... genTypes>
	ForceRegistration<genTypes...>::ForceRegistration(Particle& p_particle, auto p_forceGenerator) noexcept
		:particle_{p_particle}, forceGen_{p_forceGenerator}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	ForceRegistration<genTypes...>& 
	ForceRegistration<genTypes...>::operator=(const ForceRegistration& p_fReg) noexcept
	{
		if(this == &p_fReg)
			return *this;

		particle_ = p_fReg.particle_;
		forceGen_ = p_fReg.forceGen_;

		return *this;
	}

	//=========================================================================
	//FORCE REGISTRY
	//=========================================================================

	template<typename... genTypes>
	void
	ForceRegistry<genTypes...>::add(Particle& p_particle, auto& p_fGen) noexcept
	{
		registrations_.emplace_back(ForceRegistration<genTypes...>{p_particle, &p_fGen});
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	template<typename generatorType>
	void
	ForceRegistry<genTypes...>::remove(Particle& p_particle) noexcept
	{
		for(auto i = 0; i < registrations_.size(); ++i)
		{
			if(&registrations_[i].particle_ == &p_particle)
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
	ForceRegistry<genTypes...>::clear() noexcept
	{
		registrations_.clear();
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	void
	ForceRegistry<genTypes...>::updateForces(const float p_deltaTime) noexcept
	{
		const uint32_t numberThreads_{ std::thread::hardware_concurrency() - 1 };

		std::vector<std::thread> myThreads;
		myThreads.reserve(numberThreads_);

		const size_t length{ uint32_t(std::trunc(registrations_.size() / numberThreads_)) };
		size_t start{ 0 }, end{ length };

		for (uint32_t i = 0; i < numberThreads_; ++i)
		{
			if (i == numberThreads_ - 1)
				end = registrations_.size();

			myThreads.emplace_back(std::thread(&ForceRegistry<genTypes...>::updateForcesMultithreaded, std::reference_wrapper(*this), p_deltaTime, start, end));

			start = end;
			end = length * (i + 2);
		}

		for (uint32_t i = 0; i < numberThreads_; ++i)
		{
			myThreads[i].join();
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template<typename... genTypes>
	void
	ForceRegistry<genTypes...>::updateForcesMultithreaded(const float p_deltaTime, const size_t p_start, const size_t p_end) noexcept
	{
		for (size_t i = p_start; i < p_end; ++i)
		{
			const auto& r = registrations_[i];
			Particle& p = r.particle_;
			auto& force = r.forceGen_;

			std::visit([&](auto& force)
				{
					force->updateForce(p, p_deltaTime);
				}, force);
		}
	}
}//namespace Ocacho::Physics::MassAggregate