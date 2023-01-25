#include "Physics.hpp"

namespace Ocacho::Physics
{
	Vector3
	TransformPointToWorld(const Matrix4& p_transformMat, const Vector3& p_point) noexcept
	{
		Vector3 worldPt;

		Vector4 auxPt (p_point.x, p_point.y, p_point.z, 1.f);

		worldPt = p_transformMat * auxPt;

		return worldPt;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	Vector3
	TransformDirToWorld(const Matrix4& p_transformMat, const Vector3& p_dir) noexcept
	{
		Vector3 worldDir;

		Vector4 auxDir (p_dir.x, p_dir.y, p_dir.z, 1.f);

		worldDir = p_transformMat * auxDir;

		return worldDir;
	}
}//namespace Ocacho::Physics

namespace Ocacho::Physics::MassAggregate
{
	void
	ClearForcesParticle(Vector3& p_forceAccum) noexcept
	{
		p_forceAccum.x = 0;
		p_forceAccum.y = 0;
		p_forceAccum.z = 0;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	IntegrateParticle(MassAggregate::Particle& p_particle, const float p_deltaTime) noexcept
	{
		//OG_ENGINE_ASSERT_BREAK(p_deltaTime > 0.0f, "p_deltaTime duration == 0.0f");

		p_particle.position += p_particle.velocity * p_deltaTime;

		Vector3 resultingAcceleration = p_particle.acceleration;
		resultingAcceleration += (p_particle.forceAccum * p_particle.inverseMass);

		p_particle.velocity += resultingAcceleration * p_deltaTime;

		p_particle.velocity *= std::pow(p_particle.damping, p_deltaTime);

		ClearForcesParticle(p_particle.forceAccum);
	}
}//namespace Ocacho::Physics::MassAggregate

namespace Ocacho::Physics::RigidBody
{
	void
	ClearAccumulator(Vector3& p_forceAccum, Vector3& p_torqueAccum) noexcept
	{
		//Clear the accumulated forces
		p_forceAccum.x = 0;
		p_forceAccum.y = 0;
		p_forceAccum.z = 0;

		//Clear the accumulated torque
		p_torqueAccum.x = 0;
		p_torqueAccum.y = 0;
		p_torqueAccum.z = 0;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	IntegrateRigidBody(RigidBody& p_rigidBody, const float p_deltaTime) noexcept
	{
		//Calculate the linear acceleration from the forces applied to the RigidBody
		p_rigidBody.lastFrameAccel = p_rigidBody.acceleration;
		p_rigidBody.lastFrameAccel += p_rigidBody.forceAccum * p_rigidBody.inverseMass;

		//Calculate the angular acceleration from the torques applied to the RigidBody
		Vector3 angularAccel = p_rigidBody.invInertiaTensorWorld * p_rigidBody.torqueAccum;

		//Update the velocities, linear and angular.
		p_rigidBody.velocity += p_rigidBody.lastFrameAccel * p_deltaTime;
		p_rigidBody.rotation += angularAccel * p_deltaTime;

		//Impose drag to reduce velocity
		p_rigidBody.velocity *= std::pow(p_rigidBody.linDamping, p_deltaTime);
		p_rigidBody.rotation *= std::pow(p_rigidBody.angDamping, p_deltaTime);

		//Update the position and rotation of the RigidBody
		p_rigidBody.position += p_rigidBody.velocity * p_deltaTime;

		//Este es mi planteamiento
		Quaternion rot = Quaternion(p_rigidBody.rotation * p_deltaTime);
		p_rigidBody.orientation *= rot;

		CalculateDerivedData(p_rigidBody);

		ClearAccumulator(p_rigidBody.forceAccum, p_rigidBody.torqueAccum);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CalculateTransformMatrix(Matrix4& p_transformMatrix
		, const Vector3& p_position
		, const Quaternion& p_orientation) noexcept
	{
		p_transformMatrix = Matrix4(1.f);

		p_transformMatrix *= glm::mat4_cast(p_orientation);

		p_transformMatrix = glm::translate(p_transformMatrix, p_position);

		/*
		En glm las matrices se cargan en columnas en lugar de en filas por eso pone los valores en otros sitios HOSTIA (Rodro aprueba este insulto)
		*/
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	SetInertiaTensor(Matrix3 p_inertiaTensor
		, Matrix3& p_inverseInertiaTensor) noexcept
	{
		p_inverseInertiaTensor = glm::inverse(p_inertiaTensor);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	TransformInertiaTensor(Matrix3& p_invInertTensWorld
		, const Matrix3& p_invInertTensLocal
		, const Matrix4& p_transfMat) noexcept
	{
		p_invInertTensWorld *= p_invInertTensLocal;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CalculateDerivedData(RigidBody& p_rigidBody) noexcept
	{
		p_rigidBody.orientation = glm::normalize(p_rigidBody.orientation);

		CalculateTransformMatrix(p_rigidBody.transformMatrix
		, p_rigidBody.position
		, p_rigidBody.orientation);

		TransformInertiaTensor(p_rigidBody.invInertiaTensorWorld
		, p_rigidBody.inverseInertiaTensor
		, p_rigidBody.transformMatrix);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	AddForceToRigidBody(const Vector3& p_force, RigidBody& p_rigidBody) noexcept
	{
		p_rigidBody.forceAccum += p_force;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	AddTorqueToRigidBody(const Vector3& p_torque, RigidBody& p_rigidBody) noexcept
	{
		p_rigidBody.torqueAccum += p_torque;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	AddForceAtLocalPoint(const Vector3& p_force
		, const Vector3& p_point
		, RigidBody& p_rigidBody) noexcept
	{
		Vector3 worldPoint = TransformPointToWorld(p_rigidBody.transformMatrix, p_point);
		AddForceAtWorldPoint(p_force, worldPoint, p_rigidBody);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	AddForceAtWorldPoint(const Vector3& p_force
		, const Vector3& p_point
		, RigidBody& p_rigidBody) noexcept
	{
		//Get the point relative to the center of mass
		Vector3 pt = p_point - p_rigidBody.position;

		p_rigidBody.forceAccum += p_force;
		p_rigidBody.torqueAccum += pt * p_force;
	}
}//namespace Ocacho::Physics::RigidBody