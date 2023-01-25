#include "CollideFine.hpp"

//TEST
#include <cstdio>
#include <glm/gtx/io.hpp>
//TEST

namespace Ocacho::Physics::RigidBody
{
	//=========================================================================
	//COLLISION PRIMITIVE
	//=========================================================================

	void
	CollisionPrimitive::calculateInternals() noexcept
	{
		if(body_)
			transform_ = body_->transformMatrix * offset_;
		else
			transform_ = offset_;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	Vector3
	CollisionPrimitive::getAxis(uint8_t i) const noexcept
	{
		return Vector3 { transform_[i][0]
						, transform_[i][1]
						, transform_[i][2] };
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionPrimitive::resetValues() noexcept
	{
		body_ = nullptr;
		offset_ = Matrix4(1.f);
		transform_ = Matrix4(1.f);
	}

	//=========================================================================
	//COLLISION DETECTOR
	//=========================================================================

	uint32_t
	CollisionDetector::sphereAndSphere(const CollisionSphere& p_first, const CollisionSphere& p_second
		, CollisionData* p_colData) const noexcept
	{
		if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return 0;

		const Vector3 posFirst  = p_first.getAxis(3);
		const Vector3 posSecond = p_second.getAxis(3);

		//Find the middle position between the spheres and get the length
		const Vector3 midPosition = posFirst - posSecond;
		const float distance = glm::length(midPosition);

		//If there is no collision return
		if(distance <= 0 || distance >= (p_first.radius_ + p_second.radius_)) return 0;

		const Vector3 contNormal = glm::normalize(midPosition);

		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;

		contact->contactNormal_ = contNormal;
		contact->contactPoint_ = posFirst + midPosition*.5f;
		contact->penetration_ = p_first.radius_ + p_second.radius_ - distance;

		contact->setBodyData(p_first.body_, p_second.body_
			, p_colData->friction_, p_colData->restitution_);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::sphereAndHalfSpace(const CollisionSphere& p_sphere, const CollisionPlane& p_plane
		, CollisionData* p_colData) const noexcept
	{
		if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return 0;

		const Vector3 spherePosition = p_sphere.getAxis(3);

		const float spherePlaneDistance = glm::dot(p_plane.normal_, spherePosition) - p_sphere.radius_ - p_plane.offset_;

		if(spherePlaneDistance >= 0) return 0;

		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;

		contact->contactNormal_ = p_plane.normal_;
		contact->penetration_ = -spherePlaneDistance;
		contact->contactPoint_ = 
			spherePosition - p_plane.normal_ * (spherePlaneDistance + p_sphere.radius_);
		
		contact->setBodyData(p_sphere.body_, nullptr
			, p_colData->friction_, p_colData->restitution_);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::sphereAndTruePlane(const CollisionSphere& p_sphere, const CollisionPlane& p_plane
		, CollisionData* p_colData) const noexcept
	{
		if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return 0;

		const Vector3 spherePosition = p_sphere.getAxis(3);

		const Vector3 spherePlaneVector = p_plane.normal_ * spherePosition - p_plane.offset_;
		const float spherePlaneDistance = glm::length(spherePlaneVector);

		if(spherePlaneDistance*spherePlaneDistance > p_sphere.radius_*p_sphere.radius_)
			return 0;
		
		Vector3 spherePlaneDir = p_plane.normal_;
		float penetration = -spherePlaneDistance;

		//Check at which side of the plane is the sphere
		if(spherePlaneDistance < 0)
		{
			spherePlaneDir *= -1;
			penetration *= -1;
		}

		penetration += p_sphere.radius_;

		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;

		contact->contactNormal_ = spherePlaneDir;
		contact->penetration_ = penetration;
		contact->contactPoint_ = 
			spherePosition - p_plane.normal_ * spherePlaneDistance;
		
		contact->setBodyData(p_sphere.body_, nullptr
			, p_colData->friction_, p_colData->restitution_);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::boxAndHalfSpace(const CollisionBox& p_box, const CollisionPlane& p_plane
		, CollisionData* p_colData) const noexcept
	{
		if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return 0;

		if(!IntersectionTests::boxAndHalfSpace(p_box, p_plane)) return 0;

		uint32_t contactsGenerated = 0;
		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);

		for(uint8_t i=0; i < 8; ++i)
		{
			Vector3 vertexPos {boxMultCombinations[i].x, boxMultCombinations[i].y, boxMultCombinations[i].z};
			vertexPos = vertexPos * p_box.halfSize_;
			
			Vector4 vertexPos4 {vertexPos.x, vertexPos.y, vertexPos.z, 1.f};
			vertexPos4 = p_box.transform_ * vertexPos4;
			vertexPos = Vector3(vertexPos4.x, vertexPos4.y, vertexPos4.z); 

			float vertexDistance = glm::dot(vertexPos, p_plane.normal_);

			if(vertexDistance <= p_plane.offset_)
			{
				fillBoxAndHalfSpaceContact(contact, p_colData
					, p_box, p_plane
					, vertexDistance, vertexPos);

				contactsGenerated++;
				if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return contactsGenerated;

				//Get the next contact in the array to fill
				contact = &(p_colData->contactList_[p_colData->currentContact_]);
			}
		}

		return contactsGenerated;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::boxAndSphere(const CollisionBox& p_box, const CollisionSphere& p_sphere
		, CollisionData* p_colData) const noexcept
	{
		if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return 0;

		Vector3 centre {p_sphere.getAxis(3)};
		Vector4 relCentre4 {centre, 1.f};
		relCentre4 = glm::inverse(p_box.transform_) * relCentre4;
		Vector3 relCentre {relCentre4};

		if(!isBoxAndSphereColliding(relCentre, p_sphere.radius_, p_box.halfSize_))
			return 0;

		//Clamp each coordinate to the box
		Vector3 closestPt = clampSphereBoxCoordinates(relCentre, p_box.halfSize_);

		//Check we're in contact
		float dist = glm::length2(closestPt - relCentre);
		if(dist > p_sphere.radius_ * p_sphere.radius_) return 0;

		Vector4 closestPtWorld4 {closestPt, 1};
		closestPtWorld4 = p_box.transform_ * closestPtWorld4;
		Vector3 closestPtWorld {closestPtWorld4};

		fillBoxAndSphereContact(p_colData
			, closestPtWorld, centre, dist
			, p_box, p_sphere, relCentre);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::boxAndPoint(const CollisionBox& p_box, const Vector3& p_point
		, CollisionData* p_colData) const noexcept
	{
		if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return 0;

		Vector4 relPt4 {p_point[0], p_point[1], p_point[2], 1};
		relPt4 = glm::inverse(p_box.transform_) * relPt4;

		Vector3 relPt {relPt4[0], relPt4[1], relPt4[2]};
		Vector3 normal {};

		//Check in which axis the penetration is the least deep
		float minDepth = p_box.halfSize_.x - std::abs(relPt.x);
		if(minDepth < 0) return 0;
		normal = p_box.getAxis(0) * ((relPt.x < 0)? -1.f : 1.f);

		float depth = p_box.halfSize_.y - std::abs(relPt.y);
		if(depth < 0) return 0;
		else if(depth < minDepth)
		{
			minDepth = depth;
			normal = p_box.getAxis(1) * ((relPt.y < 0)? -1.f : 1.f);
		}

		depth = p_box.halfSize_.z - std::abs(relPt.z);
		if(depth < 0) return 0;
		else if(depth < minDepth)
		{
			minDepth = depth;
			normal = p_box.getAxis(2) * ((relPt.z < 0)? -1.f : 1.f);
		}

		fillBoxAndPointContact(p_colData
			, normal, p_point, minDepth
			, p_box);
		
		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::boxAndBox(const CollisionBox& p_first, const CollisionBox& p_second
		, CollisionData* p_colData) const noexcept
	{
		if((p_colData->contactsLeft_ - p_colData->currentContact_) == 0) return 0;
		
		if(!IntersectionTests::boxAndBox(p_first, p_second)) return 0;

		Vector3 toCentre = p_second.getAxis(3) - p_first.getAxis(3);

		float pen = FLT_MAX;
		uint32_t best = UINT32_MAX;

		//First check the individual axis for each box
		tryIndividualAxisBoxAndBox(p_first, p_second
			, toCentre, pen, best);

		uint32_t bestSingleAxis = best;

		//Now check the perpendicular result between each pair of axes
		tryPerpendicularAxisBoxAndBox(p_first, p_second
			, toCentre, pen, best);

		if(best == UINT32_MAX) return 0;

		if(best < 3)
		{
			fillPointFaceBoxBox(p_first, p_second, toCentre, p_colData, best, pen);
			return 1;
		}
		else if(best < 6)
		{
			fillPointFaceBoxBox(p_second, p_first, toCentre*-1.f, p_colData, best-3, pen);
			return 1;
		}
		else
		{
			best -= 6;
			uint32_t firstAxisIndex = best / 3;
			uint32_t secondAxisIndex = best % 3;
			Vector3 firstAxis = p_first.getAxis(firstAxisIndex);
			Vector3 secondAxis = p_second.getAxis(secondAxisIndex);
			Vector3 axis = glm::cross(firstAxis, secondAxis);
			axis = glm::normalize(axis);

			if(glm::dot(axis, toCentre) > 0) axis = axis * -1.f;

			Vector3 ptOnFirstEdge = p_first.halfSize_;
			Vector3 ptOnSecondEdge = p_second.halfSize_;

			for(uint8_t i = 0; i < 3; ++i)
			{
				if(i == firstAxisIndex) ptOnFirstEdge[i] = 0;
				else if(glm::dot(p_first.getAxis(i), axis) > 0) ptOnFirstEdge[i] = -ptOnFirstEdge[i];

				if(i == secondAxisIndex) ptOnSecondEdge[i] = 0;
				else if(glm::dot(p_second.getAxis(i), axis) > 0) ptOnSecondEdge[i] = -ptOnSecondEdge[i];
			}

			Vector4 ptOnFirstEdge4 {ptOnFirstEdge.x, ptOnFirstEdge.y, ptOnFirstEdge.z, 1.f};
			Vector4 ptOnSecondEdge4 {ptOnSecondEdge.x, ptOnSecondEdge.y, ptOnSecondEdge.z, 1.f};

			ptOnFirstEdge4 = p_first.transform_ * ptOnFirstEdge4;
			ptOnSecondEdge4 = p_second.transform_ * ptOnSecondEdge4;
			ptOnFirstEdge = Vector3(ptOnFirstEdge4);
			ptOnSecondEdge = Vector3(ptOnSecondEdge4);

			Vector3 vertex = contactPoint(ptOnFirstEdge, firstAxis, p_first.halfSize_[firstAxisIndex]
				, ptOnSecondEdge, secondAxis, p_second.halfSize_[secondAxisIndex]
				, bestSingleAxis > 2);

			fillBoxAndBoxContact(p_colData
				, pen, axis, vertex
				, p_first, p_second);
			
			return 1;
		}

		return 0;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionDetector::fillBoxAndHalfSpaceContact(Contact* p_contact, CollisionData* p_colData
		, const CollisionBox& p_box, const CollisionPlane& p_plane
		, const float p_vertexDistance, const Vector3& p_vertexPos) const noexcept
	{
		p_contact->contactPoint_ = p_plane.normal_;
		p_contact->contactPoint_ *= (p_vertexDistance-p_plane.offset_);
		p_contact->contactPoint_ += p_vertexPos;
		p_contact->contactNormal_ = p_plane.normal_;
		p_contact->penetration_ = p_plane.offset_ - p_vertexDistance;

		p_contact->setBodyData(p_box.body_, nullptr
			, p_colData->friction_, p_colData->restitution_);

		p_colData->currentContact_++;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	bool
	CollisionDetector::isBoxAndSphereColliding(const Vector3& p_relCentre
		, const float p_radius, const Vector3& p_halfSize) const noexcept
	{
		return !(std::abs(p_relCentre.x) - p_radius > p_halfSize.x
			|| std::abs(p_relCentre.y) - p_radius > p_halfSize.y
			|| std::abs(p_relCentre.z) - p_radius > p_halfSize.z);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionDetector::fillBoxAndSphereContact(CollisionData* p_colData
		, const Vector3& p_closestPtWorld, const Vector3& p_centre, const float p_dist
		, const CollisionBox& p_box, const CollisionSphere& p_sphere,
		Vector3 relCentre) const noexcept
	{
		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;

		//TEST
		float penetration = p_sphere.radius_ - std::sqrt(p_dist);
		//TEST

		if(glm::length2(p_closestPtWorld - p_centre) == 0.f)
		{
			//contact->contactNormal_ = p_closestPtWorld - p_centre;
			Vector3 totalVel {0.f};
			if(p_box.body_) totalVel += p_box.body_->velocity;
			if(p_sphere.body_) totalVel += p_sphere.body_->velocity;

			contact->contactNormal_ = glm::normalize(totalVel);
		}
		else contact->contactNormal_ = glm::normalize(p_closestPtWorld - p_centre);
		
		//TEST
		if(penetration >= 0.2f)
		{
			std::cout << "PENETRATION: " << penetration << "\n";
			printf("p_closestPtWorld: [%.10f, %.10f, %.10f]\n", p_closestPtWorld.x, p_closestPtWorld.y, p_closestPtWorld.z);
			printf("p_centre: [%.10f, %.10f, %.10f]\n", p_centre.x, p_centre.y, p_centre.z);
			printf("relCentre: [%.10f, %.10f, %.10f]\n", relCentre.x, relCentre.y, relCentre.z);
			std::cout << "contactNormal_: [" << contact->contactNormal_.x << ", " << contact->contactNormal_.y << 
			", " << contact->contactNormal_.x << "]\n";
		}
		//TEST

		contact->contactPoint_ = p_closestPtWorld;
		contact->penetration_ = p_sphere.radius_ - std::sqrt(p_dist);

		contact->setBodyData(p_box.body_, p_sphere.body_
			, p_colData->friction_, p_colData->restitution_);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionDetector::fillBoxAndPointContact(CollisionData* p_colData
		, const Vector3& p_normal, const Vector3& p_point, const float p_minDepth
		, const CollisionBox& p_box) const noexcept
	{
		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;
		contact->contactNormal_ = p_normal;
		contact->contactPoint_ = p_point;
		contact->penetration_ = p_minDepth;

		contact->setBodyData(p_box.body_, nullptr
			, p_colData->friction_, p_colData->restitution_);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	Vector3
	CollisionDetector::clampSphereBoxCoordinates(const Vector3& p_relCentre
		, const Vector3& p_halfSize) const noexcept
	{
		Vector3 closestPt {0,0,0};

		//Clamp each coordinate to the box
		closestPt.x = clampSphereBoxAxis(p_relCentre.x, p_halfSize.x);
		closestPt.y = clampSphereBoxAxis(p_relCentre.y, p_halfSize.y);
		closestPt.z = clampSphereBoxAxis(p_relCentre.z, p_halfSize.z);

		return closestPt;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	CollisionDetector::clampSphereBoxAxis(const float p_relCentreAxis
		, const float p_halfSizeAxis) const noexcept
	{
		float dist;

		dist = p_relCentreAxis;
		if(dist > p_halfSizeAxis) dist = p_halfSizeAxis;
		if(dist < -p_halfSizeAxis) dist = -p_halfSizeAxis;
		
		return dist;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	CollisionDetector::penetrationOnAxis(const CollisionBox& p_first, const CollisionBox& p_second
		, const Vector3& p_axis, const Vector3& p_toCentre) const noexcept
	{
		float firstProj = projectionAxisBoxAndBox(p_first, p_axis);

		float secondProj = projectionAxisBoxAndBox(p_second, p_axis);

		float distance = std::abs(glm::dot(p_toCentre, p_axis));

		return firstProj + secondProj - distance;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	CollisionDetector::projectionAxisBoxAndBox(const CollisionBox& p_box, const Vector3& p_axis) const noexcept
	{
		return glm::dot(p_box.halfSize_.x, std::abs(glm::dot(p_axis, p_box.getAxis(0) )))
			+ glm::dot(p_box.halfSize_.y, std::abs(glm::dot(p_axis, p_box.getAxis(1) )))
			+ glm::dot(p_box.halfSize_.z, std::abs(glm::dot(p_axis, p_box.getAxis(2) )));
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	bool
	CollisionDetector::tryAxis(const CollisionBox& p_first, const CollisionBox& p_second
		, Vector3 p_axis, const Vector3& p_toCentre, uint32_t p_index
		, float& p_smallestPenetration, uint32_t& p_smallestCase) const noexcept
	{
		if(glm::length2(p_axis) < 0.0001f) return true;
		p_axis = glm::normalize(p_axis);

		float penetration = penetrationOnAxis(p_first, p_second, p_axis, p_toCentre);

		if(penetration < 0) return false;
		if(penetration < p_smallestPenetration)
		{
			p_smallestPenetration = penetration;
			p_smallestCase = p_index;
		}

		return true;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionDetector::tryIndividualAxisBoxAndBox(const CollisionBox& p_first, const CollisionBox& p_second
		, const Vector3& p_toCentre, float& p_pen, uint32_t& p_best) const noexcept
	{
		tryAxis(p_first, p_second, p_first.getAxis(0), p_toCentre, 0, p_pen, p_best);
		tryAxis(p_first, p_second, p_first.getAxis(1), p_toCentre, 1, p_pen, p_best);
		tryAxis(p_first, p_second, p_first.getAxis(2), p_toCentre, 2, p_pen, p_best);

		tryAxis(p_first, p_second, p_second.getAxis(0), p_toCentre, 3, p_pen, p_best);
		tryAxis(p_first, p_second, p_second.getAxis(1), p_toCentre, 4, p_pen, p_best);
		tryAxis(p_first, p_second, p_second.getAxis(2), p_toCentre, 5, p_pen, p_best);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionDetector::tryPerpendicularAxisBoxAndBox(const CollisionBox& p_first, const CollisionBox& p_second
		, const Vector3& p_toCentre, float& p_pen, uint32_t& p_best) const noexcept
	{
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(0), p_second.getAxis(0)), p_toCentre, 6, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(0), p_second.getAxis(1)), p_toCentre, 7, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(0), p_second.getAxis(2)), p_toCentre, 8, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(1), p_second.getAxis(0)), p_toCentre, 9, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(1), p_second.getAxis(1)), p_toCentre, 10, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(1), p_second.getAxis(2)), p_toCentre, 11, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(2), p_second.getAxis(0)), p_toCentre, 12, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(2), p_second.getAxis(1)), p_toCentre, 13, p_pen, p_best);
		tryAxis(p_first, p_second, glm::cross(p_first.getAxis(2), p_second.getAxis(2)), p_toCentre, 14, p_pen, p_best);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionDetector::fillPointFaceBoxBox(const CollisionBox& p_first, const CollisionBox& p_second
		, const Vector3& p_toCentre, CollisionData* p_colData
		, uint32_t p_best, float p_pen) const noexcept
	{
		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;

		Vector3 normal = p_first.getAxis(p_best);
		if(glm::dot(p_first.getAxis(p_best), p_toCentre) > 0)
		{
			normal = normal * -1.f;
		}

		Vector4 vertex4 {p_second.halfSize_.x, p_second.halfSize_.y, p_second.halfSize_.z, 1.f};
		if(glm::dot(p_second.getAxis(0), normal) < 0) vertex4[0] = -vertex4[0];
		if(glm::dot(p_second.getAxis(1), normal) < 0) vertex4[1] = -vertex4[1];
		if(glm::dot(p_second.getAxis(2), normal) < 0) vertex4[2] = -vertex4[2];

		vertex4 = p_second.transform_ * vertex4;
		Vector3 vertex {vertex4.x, vertex4.y, vertex4.z};

		contact->contactNormal_ = glm::normalize(normal);
		contact->penetration_ = p_pen;
		contact->contactPoint_ = vertex;
		contact->setBodyData(p_first.body_, p_second.body_
			, p_colData->friction_, p_colData->restitution_);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	Vector3
	CollisionDetector::contactPoint(
		const Vector3& p_ptOnFirstEdge, const Vector3& p_firstAxis, float p_firstAxisHalfSize
		, const Vector3& p_ptOnSecondEdge, const Vector3& p_secondAxis, float p_secondAxisHalfSize
		, bool p_useOne) const noexcept
	{
		Vector3 toSt, cFirst, cSecond;
		float dpStaFirst, dpStaSecond, distFirstSecond, sqLenFirst, sqLenSecond;
		float denom, mua, mub;

		sqLenFirst = glm::length2(p_firstAxis);
		sqLenSecond = glm::length2(p_secondAxis);
		distFirstSecond = glm::dot(p_secondAxis, p_firstAxis);

		toSt = p_ptOnFirstEdge - p_ptOnSecondEdge;
		dpStaFirst = glm::dot(p_firstAxis, toSt);
		dpStaSecond = glm::dot(p_secondAxis, toSt);

		denom = (sqLenFirst * sqLenSecond) - (distFirstSecond * distFirstSecond);
		
		if(std::abs(denom) < 0.0001f) return p_useOne? p_ptOnFirstEdge : p_ptOnSecondEdge;

		mua = (distFirstSecond * dpStaSecond - sqLenSecond * dpStaFirst) / denom;
		mub = (sqLenFirst * dpStaSecond - distFirstSecond * dpStaFirst) / denom;

		if(mua > p_firstAxisHalfSize || mua < -p_firstAxisHalfSize
			|| mub > p_secondAxisHalfSize || mub < -p_secondAxisHalfSize)
		{
			return p_useOne? p_ptOnFirstEdge : p_ptOnSecondEdge;
		}
		else
		{
			cFirst = p_ptOnFirstEdge + p_firstAxis * mua;
			cSecond = p_ptOnSecondEdge + p_secondAxis * mub;

			return cFirst * 0.5f + cSecond * 0.5f;
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	void
	CollisionDetector::fillBoxAndBoxContact(CollisionData* p_colData
		, const float p_pen, const Vector3& p_axis, const Vector3& p_vertex
		, const CollisionBox& p_first, const CollisionBox& p_second) const noexcept
	{
		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;

		contact->penetration_ = p_pen;
		contact->contactNormal_ = p_axis;
		contact->contactPoint_ = p_vertex;
		contact->setBodyData(p_first.body_, p_second.body_
			, p_colData->friction_, p_colData->restitution_);
	}

	//=========================================================================
	//INTERSECTION TESTS
	//=========================================================================

	bool
	IntersectionTests::boxAndSphere(const CollisionBox& p_box, const CollisionSphere& p_sphere) noexcept
	{
		Vector3 centre {p_sphere.getAxis(3)};
		Vector4 relCentre4 {centre, 1.f};
		relCentre4 = glm::inverse(p_box.transform_) * relCentre4;
		Vector3 relCentre {relCentre4};

		return !(std::abs(relCentre.x) - p_sphere.radius_ > p_box.halfSize_.x
			|| std::abs(relCentre.y) - p_sphere.radius_ > p_box.halfSize_.y
			|| std::abs(relCentre.z) - p_sphere.radius_ > p_box.halfSize_.z);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	bool
	IntersectionTests::boxAndHalfSpace(const CollisionBox& p_box, const CollisionPlane& p_plane) noexcept
	{
		float projection = transformToAxis(p_box, p_plane.normal_);

		float boxDistance = glm::dot(p_plane.normal_, p_box.getAxis(3)) - projection;

		return boxDistance <= p_plane.offset_;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	bool
	IntersectionTests::boxAndBox(const CollisionBox& p_first, const CollisionBox& p_second) noexcept
	{
		Vector3 firstCentre {p_first.getAxis(3)}, secondCentre {p_second.getAxis(3)};
		float min_x, max_x, min_y, max_y, min_z, max_z;
		float other_min_x, other_max_x, other_min_y, other_max_y, other_min_z, other_max_z;

		min_x = firstCentre.x - p_first.halfSize_.x;
		max_x = firstCentre.x + p_first.halfSize_.x;
		min_y = firstCentre.y - p_first.halfSize_.y;
		max_y = firstCentre.y + p_first.halfSize_.y;
		min_z = firstCentre.z - p_first.halfSize_.z;
		max_z = firstCentre.z + p_first.halfSize_.z;

		other_min_x = secondCentre.x - p_second.halfSize_.x;
		other_max_x = secondCentre.x + p_second.halfSize_.x;
		other_min_y = secondCentre.y - p_second.halfSize_.y;
		other_max_y = secondCentre.y + p_second.halfSize_.y;
		other_min_z = secondCentre.z - p_second.halfSize_.z;
		other_max_z = secondCentre.z + p_second.halfSize_.z;

		bool overlapping = (min_x <= other_max_x && max_x >= other_min_x) &&
						   (min_y <= other_max_y && max_y >= other_min_y) &&
						   (min_z <= other_max_z && max_z >= other_min_z);

		return overlapping;

		/*Vector3 toCentre = p_second.getAxis(3) - p_first.getAxis(3);

		return (
			   overlapOnAxis(p_first, p_second, p_first.getAxis(0), toCentre)
			&& overlapOnAxis(p_first, p_second, p_first.getAxis(1), toCentre)
			&& overlapOnAxis(p_first, p_second, p_first.getAxis(2), toCentre)

			&& overlapOnAxis(p_first, p_second, p_second.getAxis(0), toCentre)
			&& overlapOnAxis(p_first, p_second, p_second.getAxis(1), toCentre)
			&& overlapOnAxis(p_first, p_second, p_second.getAxis(2), toCentre)

			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(0), p_second.getAxis(0)), toCentre)
			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(0), p_second.getAxis(1)), toCentre)
			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(0), p_second.getAxis(2)), toCentre)

			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(1), p_second.getAxis(0)), toCentre)
			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(1), p_second.getAxis(1)), toCentre)
			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(1), p_second.getAxis(2)), toCentre)

			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(2), p_second.getAxis(0)), toCentre)
			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(2), p_second.getAxis(1)), toCentre)
			&& overlapOnAxis(p_first, p_second, glm::cross(p_first.getAxis(2), p_second.getAxis(2)), toCentre)
		);*/
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	bool
	IntersectionTests::overlapOnAxis(const CollisionBox& p_first, const CollisionBox& p_second
		, const Vector3& p_axis, const Vector3& p_toCentre) noexcept
	{
		float firstProjection = transformToAxis(p_first, p_axis);
		float secondProjection = transformToAxis(p_second, p_axis);

		float distance = std::abs(glm::dot(p_toCentre, p_axis));

		return (distance < firstProjection + secondProjection);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	IntersectionTests::transformToAxis(const CollisionBox& p_box, const Vector3& p_axis) noexcept
	{
		return p_box.halfSize_.x * std::abs(glm::dot( p_axis, p_box.getAxis(0) ))
			 + p_box.halfSize_.y * std::abs(glm::dot( p_axis, p_box.getAxis(1) ))
			 + p_box.halfSize_.z * std::abs(glm::dot( p_axis, p_box.getAxis(2) ));
	}
}