#include <Utility/Timer.hpp>

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
		std::vector<std::thread> myThreads;
		myThreads.reserve(numberThreads_);

		const size_t length{ uint32_t(std::trunc(particles_.size() / numberThreads_))};
		size_t start{ 0 }, end{ length };

		for (uint32_t i = 0; i < numberThreads_; ++i)
		{
			if (i == numberThreads_ - 1)
				end = particles_.size();

			myThreads.emplace_back(std::thread(&ParticleManager<ForceList, ContactGenList>::integrateParticlesMultithreaded, std::reference_wrapper(*this), p_deltaTime, start, end));

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

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::integrateParticlesMultithreaded(const float p_deltaTime, const size_t p_start, const size_t p_end) noexcept
	{
		for (size_t i = p_start; i < p_end; ++i)
		{
			auto& p = particles_[i];

			if (p)
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
		Ocacho::Timer timer;
		
		static uint8_t times;

		timer.start();

		//First we apply the forces to the particles
		forceReg_.updateForces(p_deltaTime);

		//Now we integrate the particles
		integrateParticles(p_deltaTime);

		//Later generate contacts for all the contact generators
		unsigned usedContacts = generateContacts();

		//Finally we resolve all the contacts that we've generated
		contactRes_.setIterations(usedContacts * 2);
		contactRes_.resolveContacts(contactList_, p_deltaTime);

		if (times >= 20)
		{
			printf("Ellapsed time: %fms \n", float(timer.ellapsedTime())/1000000);
			times = 0;
		}

		++times;
	}

	//=========================================================================
	//METHODS TO ADD DATA
	//=========================================================================

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::addParticle(Particle& p_particle) noexcept
	{
		std::lock_guard<std::mutex> guard(particlesMutex_);
		particles_.push_back(&p_particle);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::addContactGenerator(auto p_contactGen) noexcept
	{
		std::lock_guard<std::mutex> guard(contactGenMutex_);
		contactGenVector_.push_back(p_contactGen);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList, typename ContactGenList>
	void
	ParticleManager<ForceList, ContactGenList>::addForceRegistration(Particle& p_particle, auto& p_forceGen) noexcept
	{
		std::lock_guard<std::mutex> guard(forceRegMutex_);
		forceReg_.add(p_particle, p_forceGen);
	}
};//namespace Ocacho::Physics::MassAggregate