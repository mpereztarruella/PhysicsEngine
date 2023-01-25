namespace Ocacho::Physics::RigidBody
{
	//=========================================================================
	//GRAVITY FORCE GENERATOR
	//=========================================================================

	void
	GravityForceGeneratorRB::updateForce(RigidBody& p_rigidBody, const float p_deltaTime) const noexcept
	{
		if(p_rigidBody.inverseMass == 0.0f) return;
		
		p_rigidBody.forceAccum += (gravity_ * p_rigidBody.mass);
	}

	//=========================================================================
	//DRAG FORCE GENERATOR
	//=========================================================================

	void
	DragForceGeneratorRB::updateForce(RigidBody& p_rigidBody, const float p_deltaTime) const noexcept
	{
		float dragCoeff = glm::length(p_rigidBody.velocity);
		dragCoeff = k1_ * dragCoeff + k2_ * dragCoeff * dragCoeff;

		p_rigidBody.forceAccum += -p_rigidBody.velocity * dragCoeff;
	}

	//=========================================================================
	//SPRING FORCE GENERATOR
	//=========================================================================
	
	SpringForceGeneratorRB::SpringForceGeneratorRB(Vector3 p_localConnPt
		, float p_springConstant
		, float p_restLength
		, Vector3 p_otherConnPt
		, const RigidBody& p_otherRigidBody) noexcept 
		: connectPoint_{p_localConnPt}
		, springConstant_{p_springConstant}
		, restLength_{p_restLength}
		, otherConnectPoint_{p_otherConnPt}
		, otherRigidBody_{p_otherRigidBody}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	SpringForceGeneratorRB::updateForce(RigidBody& p_rigidBody, const float p_deltaTime) noexcept
	{
		//First get the points in world space
		auto mainRbPt = TransformPointToWorld(p_rigidBody.transformMatrix, connectPoint_);
		auto otherRbPt = TransformPointToWorld(otherRigidBody_.transformMatrix, otherConnectPoint_);

		auto forceDir = mainRbPt - otherRbPt;

		auto forceMagnitude = glm::length(forceDir);
		auto totalMagnitude = std::abs(forceMagnitude - restLength_);
		totalMagnitude *= -springConstant_;

		forceDir = glm::normalize(forceDir);
		forceDir *= totalMagnitude;

		//Call the function that applies force and torque
		AddForceAtWorldPoint(forceDir
			, mainRbPt
			, p_rigidBody);
	}

	//=========================================================================
	//ANCHORED SPRING FORCE GENERATOR
	//=========================================================================

	AnchoredSpringForceGeneratorRB::AnchoredSpringForceGeneratorRB(Vector3 p_localConnPt
		, const float p_springConstant
		, const float p_restLength
		, const Vector3 p_anchoredPosition) noexcept
		: connectPoint_{p_localConnPt}
		, springConstant_{p_springConstant}
		, restLength_{p_restLength}
		, anchoredPosition_{p_anchoredPosition}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	AnchoredSpringForceGeneratorRB::updateForce(RigidBody& p_rigidBody
		, const float p_deltaTime) noexcept
	{
		//First get the point in world space
		auto mainRbPt = TransformPointToWorld(p_rigidBody.transformMatrix, connectPoint_);

		auto forceDir = mainRbPt - anchoredPosition_;

		auto forceMagnitude = glm::length(forceDir);

		auto totalMagnitude = std::abs(forceMagnitude - restLength_);
		totalMagnitude *= -springConstant_;

		forceDir = glm::normalize(forceDir);
		forceDir *= totalMagnitude;

		//Call the function that applies force and torque
		AddForceAtWorldPoint(forceDir
			, mainRbPt
			, p_rigidBody);
	}

	//=========================================================================
	//ELASTIC BUNGEE FORCE GENERATOR
	//=========================================================================

	ElasticBungeeForceGeneratorRB::ElasticBungeeForceGeneratorRB(Vector3 p_localConnPt
		, float p_springConstant
		, float p_restLength
		, Vector3 p_otherConnPt
		, const RigidBody& p_otherRigidBody) noexcept 
		: connectPoint_{p_localConnPt}
		, springConstant_{p_springConstant}
		, restLength_{p_restLength}
		, otherConnectPoint_{p_otherConnPt}
		, otherRigidBody_{p_otherRigidBody}
	{}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	ElasticBungeeForceGeneratorRB::updateForce(RigidBody& p_rigidBody
		, const float p_deltaTime) noexcept
	{
		//First get the points in world space
		auto mainRbPt = TransformPointToWorld(p_rigidBody.transformMatrix, connectPoint_);
		auto otherRbPt = TransformPointToWorld(otherRigidBody_.transformMatrix, otherConnectPoint_);

		auto forceDir = mainRbPt - otherRbPt;

		auto forceMagnitude = glm::length(forceDir);

		if(forceMagnitude < restLength_) return;

		auto totalMagnitude = std::abs(forceMagnitude - restLength_);
		totalMagnitude *= -springConstant_;

		forceDir = glm::normalize(forceDir);
		forceDir *= totalMagnitude;

		//Call the function that applies force and torque
		AddForceAtWorldPoint(forceDir
			, mainRbPt
			, p_rigidBody);
	}

	//=========================================================================
	//FAKE SPRING FORCE GENERATOR
	//=========================================================================

	FakeSpringForceGeneratorRB::FakeSpringForceGeneratorRB(Vector3 p_localConnPt
		, const float p_springConstant
		, const float p_damping
		, const Vector3 p_anchoredPosition) noexcept
		: connectPoint_{p_localConnPt}
		, springConstant_{p_springConstant}
		, damping_{p_damping}
		, anchoredPosition_{p_anchoredPosition}
	{}
	
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	FakeSpringForceGeneratorRB::updateForce(RigidBody& p_rigidBody, const float p_deltaTime) noexcept
	{
		if(p_rigidBody.inverseMass == 0.0f) return;

		//First get the point in world space
		Vector3 mainRBPt = TransformPointToWorld(p_rigidBody.transformMatrix, connectPoint_);
		
		//Calculate the distance from the particle to the anchoredPosition
		Vector3 distance = mainRBPt - anchoredPosition_;

		//Calculate the constants
		float gamma = 0.5f * std::sqrt(4*springConstant_ - damping_*damping_);

		if(gamma == 0.0f) return;

		Vector3 c = (damping_/2*gamma)*distance;
		c += p_rigidBody.velocity * (1.0f/gamma);

		//Calculate the target position
		Vector3 target = distance * std::cos(gamma * p_deltaTime) + 
						 c * std::sin(gamma * p_deltaTime);
		target *= std::exp(-0.5f * damping_ * p_deltaTime);

		//Calculate the resulting acceleration and the force
		Vector3 acceleration = (target - distance) * (1.0f / (p_deltaTime*p_deltaTime));
		acceleration -= p_rigidBody.velocity * p_deltaTime;

		Vector3 force = acceleration * p_rigidBody.mass;
		
		//Call the function that applies force and torque
		AddForceAtWorldPoint(force
			, mainRBPt
			, p_rigidBody);
	}
}//namespace Ocacho::Physics::MassAggregate