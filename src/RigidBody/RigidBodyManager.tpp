#include <Utility/Timer.hpp>

namespace Ocacho::Physics::RigidBody
{
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
		rigidBodies_.push_back(&p_rigidBody);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename ForceList>
	void
	RigidBodyManager<ForceList>::addForceRegistration(RigidBody& p_rigidBody, auto& p_forceGen) noexcept
	{
		forceReg_.add(p_rigidBody, p_forceGen);
	}
};//namespace Ocacho::Physics::RigidBody