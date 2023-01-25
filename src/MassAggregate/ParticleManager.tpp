namespace Ocacho::Physics::MassAggregate
{
	template <typename ForceList, typename ContactGenList>
	ParticleManager<ForceList, ContactGenList>::ParticleManager(unsigned p_maxContacts, unsigned p_iterations) noexcept
		: contactRes_{p_iterations} 
	{
		contactList_.reserve(p_maxContacts);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	template <typename ForceList, typename ContactGenList>
	unsigned
	ParticleManager<ForceList, ContactGenList>::generateContacts() noexcept
	{
		unsigned used = 0;

		for(auto& cGen : contactGenVector_)
		{
			std::visit([&](auto& contactGen)
			{
				used += contactGen.addContact(contactList_);
			}, cGen );

			if(contactList_.size() == contactList_.capacity())
				break;
		}

		return used;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::integrateParticles(const float p_deltaTime) noexcept
	{
		for(auto& p : particles_)
		{
			if(p)
				Ocacho::Physics::MassAggregate::IntegrateParticle(*p, p_deltaTime);
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::reset() noexcept
	{
		//Reset the force accumulator for all the particles
		for(auto& p : particles_)
		{
			if(p)
				p->forceAccum = Ocacho::Vector3(0.0f, 0.0f, 0.0f);
		}

		//Clear the contacts generated the last frame
		contactList_.clear();
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::runPhysics(const float p_deltaTime) noexcept
	{
		//First we apply the forces to the particles
		forceReg_.updateForces(p_deltaTime);

		//Now we integrate the particles
		integrateParticles(p_deltaTime);

		//Later generate contacts for all the contact generators
		unsigned usedContacts = generateContacts();

		//Finally we resolve all the contacts that we've generated
		contactRes_.setIterations(usedContacts * 2);
		contactRes_.resolveContacts(contactList_, p_deltaTime);
	}

	//=========================================================================
	//METHODS TO ADD DATA
	//=========================================================================

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::addParticle(Particle& p_particle) noexcept
	{
		particles_.push_back(&p_particle);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::addContactGenerator(auto p_contactGen) noexcept
	{
		contactGenVector_.push_back(p_contactGen);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::addForceRegistration(Particle& p_particle, auto& p_forceGen) noexcept
	{
		forceReg_.add(p_particle, p_forceGen);
	}
};//namespace Ocacho::Physics::MassAggregate