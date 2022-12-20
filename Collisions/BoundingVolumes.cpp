#include "BoundingVolumes.hpp"

namespace Ocacho::Physics::Collider
{
	//=========================================================================
	//BOUNDING SPHERE
	//=========================================================================

	BoundingSphere::BoundingSphere(const Vector3& p_centre, float p_radius) noexcept
		: centre_ {p_centre}
		, radius_ {p_radius} {}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	BoundingSphere::BoundingSphere(const BoundingSphere& p_first, const BoundingSphere& p_second) noexcept
	{
		Vector3 centreOffset = p_second.centre_ - p_first.centre_;
		float distance = glm::length2(centreOffset);
		float radiusDiff = p_second.radius_ - p_first.radius_;

		//The larger sphere encloses the small one
		if(radiusDiff*radiusDiff >= distance)
		{
			if(p_first.radius_ > p_second.radius_)
			{
				centre_ = p_first.centre_;
				radius_ = p_first.radius_;
			}
			else
			{
				centre_ = p_second.centre_;
				radius_ = p_second.radius_;
			}
		}
		else	//The spheres are partially overlapping
		{
			distance = std::sqrt(distance);
			radius_ = (distance + p_first.radius_ + p_second.radius_) / 2.f;

			centre_ = p_first.centre_;

			if (distance > 0)
			{
				centre_ += centreOffset * ((radius_ - p_first.radius_) / distance);
			}
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	bool
	BoundingSphere::overlaps(const BoundingSphere& p_other) const noexcept
	{
		float distanceSq = glm::length2(centre_ - p_other.centre_);

		float totalRadius = radius_ + p_other.radius_;

		return ( distanceSq < (totalRadius*totalRadius) );
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	BoundingSphere::getGrowth(const BoundingSphere& p_other) const noexcept
	{
		BoundingSphere newSphere(*this, p_other);

		float growth = newSphere.radius_*newSphere.radius_;
		growth -= (radius_*radius_);

		return growth;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	BoundingSphere::getVolume() const noexcept
	{
		float volume = (4/3)*PI*(radius_*radius_*radius_);

		return volume;
	}

	//=========================================================================
	//BOUNDING BOX
	//=========================================================================

	BoundingBox::BoundingBox(const Vector3& p_centre, const Vector3& p_halfExtents) noexcept
		: centre_ {p_centre}
		, halfExtents_ {p_halfExtents} {}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	BoundingBox::BoundingBox(const BoundingBox& p_first, const BoundingBox& p_second) noexcept
	{
		//TODO[Otto]: Esto hacerlo bien cuando haya tiempo de momento no es necesario

		/*Vector3 centreOffset = p_second.centre_ - p_first.centre_;
		float distance = glm::length2(centreOffset);
		float radiusDiff = p_second.radius_ - p_first.radius_;

		//The larger sphere encloses the small one
		if(radiusDiff*radiusDiff >= distance)
		{
			if(p_first.radius_ > p_second.radius_)
			{
				centre_ = p_first.centre_;
				radius_ = p_first.radius_;
			}
			else
			{
				centre_ = p_second.centre_;
				radius_ = p_second.radius_;
			}
		}
		else	//The spheres are partially overlapping
		{
			distance = std::sqrt(distance);
			radius_ = (distance + p_first.radius_ + p_second.radius_) / 2.f;

			centre_ = p_first.centre_;

			if (distance > 0)
			{
				centre_ += centreOffset * ((radius_ - p_first.radius_) / distance);
			}
		}*/
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	bool
	BoundingBox::overlaps(const BoundingBox& p_other) const noexcept
	{
		float min_x, max_x, min_y, max_y, min_z, max_z;
		float other_min_x, other_max_x, other_min_y, other_max_y, other_min_z, other_max_z;

		min_x = centre_.x - halfExtents_.x;
		max_x = centre_.x + halfExtents_.x;
		min_y = centre_.y - halfExtents_.y;
		max_y = centre_.y + halfExtents_.y;
		min_z = centre_.z - halfExtents_.z;
		max_z = centre_.z + halfExtents_.z;

		other_min_x = p_other.centre_.x - p_other.halfExtents_.x;
		other_max_x = p_other.centre_.x + p_other.halfExtents_.x;
		other_min_y = p_other.centre_.y - p_other.halfExtents_.y;
		other_max_y = p_other.centre_.y + p_other.halfExtents_.y;
		other_min_z = p_other.centre_.z - p_other.halfExtents_.z;
		other_max_z = p_other.centre_.z + p_other.halfExtents_.z;

		bool overlapping = (min_x <= other_max_x && max_x >= other_min_x) &&
						   (min_y <= other_max_y && max_y >= other_min_y) &&
						   (min_z <= other_max_z && max_z >= other_min_z);

		return overlapping;
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	BoundingBox::getGrowth(const BoundingBox& p_other) const noexcept
	{
		//TODO[Otto]: Esto hacerlo bien cuando haya tiempo de momento no es necesario

		/*BoundingBox newSphere(*this, p_other);

		float growth = newSphere.radius_*newSphere.radius_;
		growth -= (radius_*radius_);

		return growth;*/
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	float
	BoundingBox::getVolume() const noexcept
	{
		float volume = (halfExtents_.x * 2) *
					   (halfExtents_.y * 2) *
					   (halfExtents_.z * 2);

		return volume;
	}
}//namespace Ocacho::Physics::Collider