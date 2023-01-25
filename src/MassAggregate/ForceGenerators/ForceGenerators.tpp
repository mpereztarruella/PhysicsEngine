namespace Ocacho::Physics::MassAggregate
{
	//=========================================================================
	//GRAVITY FORCE GENERATOR
	//=========================================================================

	void
	GravityForceGenerator::updateForce(Particle& p_particle, const float p_deltaTime) const noexcept
	{
		if(p_particle.inverseMass == 0.0f) return;
		
		p_particle.forceAccum += (gravity * p_particle.mass);
	}

	//=========================================================================
	//DRAG FORCE GENERATOR
	//=========================================================================

	void
	DragForceGenerator::updateForce(Particle& p_particle, const float p_deltaTime) const noexcept
	{
		float dragCoeff = glm::length(p_particle.velocity);
		dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

		p_particle.forceAccum += -p_particle.velocity * dragCoeff;
	}

	//=========================================================================
	//SPRING FORCE GENERATOR
	//=========================================================================
	
	SpringForceGenerator::SpringForceGenerator(float p_springConstant, float p_restLength,
								  					    const Particle& p_otherParticle) noexcept 
		: springConstant{p_springConstant},
			restLength{p_restLength},
			otherParticle{p_otherParticle}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	SpringForceGenerator::updateForce(Particle& p_particle, const float p_deltaTime) const noexcept
	{
		auto forceDir = p_particle.position - otherParticle.position;

		auto forceMagnitude = glm::length(forceDir);
		auto totalMagnitude = std::abs(forceMagnitude - restLength);
		totalMagnitude *= -springConstant;

		forceDir = glm::normalize(forceDir);
		p_particle.forceAccum += totalMagnitude*forceDir;
	}

	//=========================================================================
	//ANCHORED SPRING FORCE GENERATOR
	//=========================================================================

	AnchoredSpringForceGenerator::AnchoredSpringForceGenerator(const float p_springConstant, const float p_restLength,
										  						const Vector3 p_anchoredPosition) noexcept
		: springConstant{p_springConstant},
		restLength{p_restLength},
		anchoredPosition{p_anchoredPosition}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	AnchoredSpringForceGenerator::updateForce(Particle& p_particle, const float p_deltaTime) const noexcept
	{
		auto forceDir = p_particle.position - anchoredPosition;

		auto forceMagnitude = glm::length(forceDir);

		auto totalMagnitude = std::abs(forceMagnitude - restLength);
		totalMagnitude *= springConstant;

		forceDir = glm::normalize(forceDir);
		forceDir *= -totalMagnitude;
		p_particle.forceAccum += forceDir;
	}

	//=========================================================================
	//ELASTIC BUNGEE FORCE GENERATOR
	//=========================================================================

	ElasticBungeeForceGenerator::ElasticBungeeForceGenerator(const float p_springConstant, const float p_restLength,
															 const Particle& p_otherParticle) noexcept
		: springConstant{p_springConstant},
		restLength{p_restLength},
		otherParticle{p_otherParticle}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	ElasticBungeeForceGenerator::updateForce(Particle& p_particle, const float p_deltaTime) const noexcept
	{
		auto forceDir = p_particle.position - otherParticle.position;

		auto forceMagnitude = glm::length(forceDir);

		if(forceMagnitude < restLength) return;

		auto totalMagnitude = std::abs(forceMagnitude - restLength);
		totalMagnitude *= -springConstant;

		forceDir = glm::normalize(forceDir);
		p_particle.forceAccum += totalMagnitude*forceDir;
	}

	//=========================================================================
	//FAKE SPRING FORCE GENERATOR
	//=========================================================================

	FakeSpringForceGenerator::FakeSpringForceGenerator(const float p_springConstant, const float p_damping,
													   const Vector3 p_anchoredPosition) noexcept
		: springConstant{p_springConstant},
		damping{p_damping},
		anchoredPosition{p_anchoredPosition}
	{}
	
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	FakeSpringForceGenerator::updateForce(Particle& p_particle, const float p_deltaTime) const noexcept
	{
		if(p_particle.inverseMass == 0.0f) return;

		//Calculate the distance from the particle to the anchoredPosition
		Vector3 position;
		position = p_particle.position - anchoredPosition;

		//Calculate the constants
		float gamma = 0.5f * std::sqrt(4*springConstant - damping*damping);

		if(gamma == 0.0f) return;

		Vector3 c = (damping/2*gamma)*position;
		c += p_particle.velocity * (1.0f/gamma);

		//Calculate the target position
		Vector3 target = position * std::cos(gamma * p_deltaTime) + 
						 c * std::sin(gamma * p_deltaTime);
		target *= std::exp(-0.5f * damping * p_deltaTime);

		//Calculate the resulting acceleration and the force
		Vector3 acceleration = (target - position) * (1.0f / (p_deltaTime*p_deltaTime));
		acceleration -= p_particle.velocity * p_deltaTime;

		p_particle.forceAccum += acceleration * p_particle.mass;
	}
}//namespace Ocacho::Physics::MassAggregate