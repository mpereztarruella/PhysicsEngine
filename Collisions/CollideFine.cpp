#include "CollideFine.hpp"

namespace Ocacho::Physics::RigidBody
{
	//=========================================================================
	//COLLISION PRIMITIVE
	//=========================================================================

	void
	CollisionPrimitive::calculateInternals() noexcept
	{
		transform_ = body_->transformMatrix * offset_;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	//TODO[Otto]: Comprobar que el getAxis va guay con las matrices column de glm
	Vector3
	CollisionPrimitive::getAxis(uint8_t i) const noexcept
	{
		return Vector3 { transform_[i][0]
						, transform_[i][1]
						, transform_[i][2] };
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	CollisionPlane::distanceToBox(const CollisionBox& p_box) const noexcept
	{
		float distanceInX = glm::dot(p_box.halfSize_.x, std::abs(glm::dot(normal_, p_box.getAxis(0) )));
		float distanceInY = glm::dot(p_box.halfSize_.y, std::abs(glm::dot(normal_, p_box.getAxis(1) )));
		float distanceInZ = glm::dot(p_box.halfSize_.z, std::abs(glm::dot(normal_, p_box.getAxis(2) )));

		float projectedDistance = distanceInX + distanceInY + distanceInZ;

		return glm::dot(normal_, (p_box.getAxis(3) - projectedDistance));
	}

	//=========================================================================
	//COLLISION DETECTOR
	//=========================================================================

	uint32_t
	CollisionDetector::sphereAndSphere(const CollisionSphere& p_first, const CollisionSphere& p_second
		, CollisionData* p_colData) const noexcept
	{
		if(p_colData->contactsLeft_ == 0) return 0;

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
		p_colData->contactsLeft_--;

		contact->contactNormal_ = contNormal;
		contact->contactPoint_ = posFirst + midPosition*.5f;
		contact->penetration_ = p_first.radius_ + p_second.radius_ - distance;

		contact->setBodyData(p_first.body_, p_second.body_, p_colData->friction_, p_colData->restitution_);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::sphereAndHalfSpace(const CollisionSphere& p_sphere, const CollisionPlane& p_plane
		, CollisionData* p_colData) const noexcept
	{
		if(p_colData->contactsLeft_ == 0) return 0;

		const Vector3 spherePosition = p_sphere.getAxis(3);

		const Vector3 spherePlaneVector = p_plane.normal_ * spherePosition - p_sphere.radius_ - p_plane.offset_;
		const float sphereDistance = glm::length(spherePlaneVector);

		if(sphereDistance >= 0) return 0;

		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;
		p_colData->contactsLeft_--;

		contact->contactNormal_ = p_plane.normal_;
		contact->penetration_ = -sphereDistance;
		contact->contactPoint_ = 
			spherePosition - p_plane.normal_ * (sphereDistance + p_sphere.radius_);
		
		contact->setBodyData(p_sphere.body_, nullptr, p_colData->friction_, p_colData->restitution_);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::sphereAndTruePlane(const CollisionSphere& p_sphere, const CollisionPlane& p_plane
		, CollisionData* p_colData) const noexcept
	{
		if(p_colData->contactsLeft_ == 0) return 0;

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
		p_colData->contactsLeft_--;

		contact->contactNormal_ = spherePlaneDir;
		contact->penetration_ = penetration;
		contact->contactPoint_ = 
			spherePosition - p_plane.normal_ * spherePlaneDistance;
		
		contact->setBodyData(p_sphere.body_, nullptr, p_colData->friction_, p_colData->restitution_);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::boxAndHalfSpace(const CollisionBox& p_box, const CollisionPlane& p_plane
		, CollisionData* p_colData) const noexcept
	{
		if(p_colData->contactsLeft_ == 0) return 0;

		if(!IntersectionTests::boxAndHalfSpace(p_box, p_plane)) return 0;

		uint32_t contactsGenerated = 0;
		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);

		for(uint8_t i=0; i < 8; ++i)
		{
			Vector4 vertexPos4 {boxMultCombinations[i].x
							, boxMultCombinations[i].y
							, boxMultCombinations[i].z
							, 1};
			vertexPos4 = vertexPos4 * p_box.transform_;
			Vector3 vertexPos {vertexPos4[0], vertexPos4[1], vertexPos4[2]}; 
			vertexPos *= p_box.halfSize_;

			float vertexDistance = glm::dot(vertexPos, p_plane.normal_);

			//TODO[Otto]: Preguntar a Fran sobre esta comprobacion (Anotaciones de la pizarra)
			if(vertexDistance <= p_plane.offset_)
			{
				fillBoxAndHalfSpaceContact(contact, p_colData
					, p_box, p_plane
					, vertexDistance, vertexPos);

				contactsGenerated++;
				if(p_colData->contactsLeft_ == 0) return contactsGenerated;
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
		Vector3 centre {p_sphere.getAxis(3)};
		Vector4 relCentre4 {centre[0], centre[1], centre[2], 1};
		relCentre4 = relCentre4 * glm::inverse(p_sphere.transform_);
		Vector3 relCentre {relCentre4[0], relCentre4[1], relCentre4[2]};

		if(isBoxAndSphereColliding(relCentre, p_sphere.radius_, p_box.halfSize_))
			return 0;

		//Clamp each coordinate to the box
		Vector3 closestPt = clampSphereBoxCoordinates(relCentre, p_box.halfSize_);

		//Check we're in contact
		float dist = glm::length2(closestPt - relCentre);
		if(dist > p_sphere.radius_ * p_sphere.radius_) return 0;

		Vector4 closestPtWorld4 {closestPt[0], closestPt[1], closestPt[2], 1};
		closestPtWorld4 = closestPtWorld4 * p_box.transform_;
		Vector3 closestPtWorld {closestPtWorld4[0], closestPtWorld4[1], closestPtWorld4[2]};

		fillBoxAndSphereContact(p_colData
			, closestPtWorld, centre, dist
			, p_box, p_sphere);

		return 1;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	uint32_t
	CollisionDetector::boxAndPoint(const CollisionBox& p_box, const Vector3& p_point
		, CollisionData* p_colData) const noexcept
	{
		Vector4 relPt4 {p_point[0], p_point[1], p_point[2], 1};
		relPt4 = relPt4 * glm::inverse(p_box.transform_);

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

		if(best == 0xffffff) return 0;

		if(best < 3)
		{
			fillPointFaceBoxBox(p_first, p_second, toCentre, p_colData, best, pen);
			return 1;
		}
		else if(best < 6)
		{
			fillPointFaceBoxBox(p_first, p_second, toCentre*-1.f, p_colData, best-3, pen);
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

			Vector4 ptOnFirstEdge4 {ptOnFirstEdge[0], ptOnFirstEdge[1], ptOnFirstEdge[2], 1.f};
			Vector4 ptOnSecondEdge4 {ptOnSecondEdge[0], ptOnSecondEdge[1], ptOnSecondEdge[2], 1.f};

			ptOnFirstEdge4 = p_first.transform_ * ptOnFirstEdge4;
			ptOnSecondEdge4 = p_second.transform_ * ptOnSecondEdge4;
			ptOnFirstEdge = ptOnFirstEdge4;
			ptOnSecondEdge = ptOnSecondEdge4;

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
		p_colData->contactsLeft_--;
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
		, const CollisionBox& p_box, const CollisionSphere& p_sphere) const noexcept
	{
		Contact* contact = &(p_colData->contactList_[p_colData->currentContact_]);
		p_colData->currentContact_++;
		p_colData->contactsLeft_--;

		contact->contactNormal_ = glm::normalize(p_closestPtWorld - p_centre);
		contact->contactPoint_ = p_closestPtWorld;
		contact->penetration_ = p_sphere.radius_ - std::sqrt(p_dist);

		contact->setBodyData(p_box.body_, p_sphere.body_
			,p_colData->friction_, p_colData->restitution_);
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
		p_colData->contactsLeft_--;
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
		else if(dist < -p_halfSizeAxis) dist = -p_halfSizeAxis;
		
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
		if(glm::length2(p_axis) < 0.0001) return true;
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
		p_colData->contactsLeft_--;

		Vector3 normal = p_first.getAxis(p_best);
		if(glm::dot(normal, p_toCentre) > 0)
		{
			normal *= -1.f;
		}

		Vector4 vertex4 {p_second.halfSize_[0], p_second.halfSize_[1], p_second.halfSize_[2], 1.f};
		if(glm::dot(p_second.getAxis(0), normal) < 0) vertex4[0] = -vertex4[0];
		if(glm::dot(p_second.getAxis(1), normal) < 0) vertex4[1] = -vertex4[1];
		if(glm::dot(p_second.getAxis(2), normal) < 0) vertex4[2] = -vertex4[2];

		vertex4 = p_second.transform_ * vertex4;
		Vector3 vertex {vertex4[0], vertex4[1], vertex4[2]};

		contact->contactNormal_ = normal;
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

		denom = glm::dot(sqLenFirst, sqLenSecond) - (distFirstSecond * distFirstSecond);
		
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
		p_colData->contactsLeft_--;

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
	IntersectionTests::boxAndHalfSpace(const CollisionBox& p_box, const CollisionPlane& p_plane) noexcept
	{
		return (p_plane.distanceToBox(p_box) <= p_plane.offset_);
	}
}