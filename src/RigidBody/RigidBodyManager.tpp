namespace Ocacho::Physics::RigidBody
{
	template <typename ForceList>
	RigidBodyManager<ForceList>::RigidBodyManager() noexcept
	{
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	//template <typename ForceList, typename ContactGenList>
	//unsigned
	//RigidBodyManager<ForceList, ContactGenList>::generateContacts() noexcept
	//{
	//	unsigned used = 0;

	//	for (auto& cGen : contactGenVector_)
	//	{
	//		std::visit([&](auto& contactGen)
	//			{
	//				used += contactGen.addContact(contactList_);
	//			}, cGen);

	//		if (contactList_.size() == contactList_.capacity())
	//			break;
	//	}

	//	return used;
	//}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList>
	void
	RigidBodyManager<ForceList>::integrateRigidBodies(const float p_deltaTime) noexcept
	{
		for (size_t i = 0; i < rigidBodies_.size(); ++i)
		{
			auto& r = rigidBodies_[i];

			if (r)
				Ocacho::Physics::RigidBody::IntegrateRigidBody(*r, p_deltaTime);
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	//template <typename ForceList, typename ContactGenList>
	//void
	//RigidBodyManager<ForceList, ContactGenList>::integrateParticlesMultithreaded(const float p_deltaTime, const size_t p_start, const size_t p_end) noexcept
	//{
	//	for (size_t i = p_start; i < p_end; ++i)
	//	{
	//		auto& p = particles_[i];

	//		if (p)
	//			Ocacho::Physics::MassAggregate::IntegrateParticle(*p, p_deltaTime);
	//	}
	//}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList>
	void
	RigidBodyManager<ForceList>::reset() noexcept
	{
		//Reset the force accumulator for all the particles
		for (auto& r : rigidBodies_)
		{
			if (r)
				r->forceAccum = Ocacho::Vector3(0.0f, 0.0f, 0.0f);
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList>
	void
	RigidBodyManager<ForceList>::runPhysics(const float p_deltaTime) noexcept
	{
		Ocacho::Timer timer;

		static uint8_t times;

		timer.start();

		//First we apply the forces to the particles
		forceReg_.updateForces(p_deltaTime);

		//Now we integrate the particles
		integrateRigidBodies(p_deltaTime);

		if (times >= 20)
		{
			printf("runPhysics time: %fms \n", float(timer.ellapsedTime()) / 1000000);
			times = 0;
		}

		++times;
	}

	//=========================================================================
	//METHODS TO ADD DATA
	//=========================================================================

	template <typename ForceList>
	void
	RigidBodyManager<ForceList>::addRigidBody(RigidBody& p_rigidBody) noexcept
	{
		//std::lock_guard<std::mutex> guard(particlesMutex_);
		rigidBodies_.push_back(&p_rigidBody);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList>
	void
	RigidBodyManager<ForceList>::addForceRegistration(RigidBody& p_rigidBody, auto& p_forceGen) noexcept
	{
		//std::lock_guard<std::mutex> guard(forceRegMutex_);
		forceReg_.add(p_rigidBody, p_forceGen);
	}
}//namespace Ocacho::Physics::RigidBody