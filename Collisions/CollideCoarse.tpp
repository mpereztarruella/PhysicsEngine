namespace Ocacho::Physics::RigidBody
{
	template <typename BoundingVolume>
	BVHNode<BoundingVolume>::BVHNode(BVHNode* p_parent, const BoundingVolume& p_volume,
		RigidBody* p_body) noexcept
		: volume_{p_volume}
		, body_{p_body} 
		, parent_{p_parent} {}
	
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename BoundingVolume>
	BVHNode<BoundingVolume>::~BVHNode()
	{
		if(parent_)
		{
			children_t sibling;

			if(parent_->children_[0].get() == this)
				sibling = std::move(parent_->children_[1]);
			else
				sibling = std::move(parent_->children_[0]);

			parent_->volume_ = sibling->volume_;
			parent_->body_ = sibling->body_;
			parent_->children_[0] = std::move(sibling->children_[0]);
			parent_->children_[1] = std::move(sibling->children_[1]);

			sibling->parent_ = nullptr;
			sibling->body_ = nullptr;

			parent_->recalculateBoundingVolume();
		}

		if(children_[0])
		{
			children_[0]->parent_ = nullptr;
		}

		if(children_[1])
		{
			children_[1]->parent_ = nullptr;
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename BoundingVolume>
	bool
	BVHNode<BoundingVolume>::isLeaf() const noexcept
	{
		return (body_ != nullptr);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename BoundingVolume>
	unsigned
	BVHNode<BoundingVolume>::getPotentialContacts(PotentialContact* p_contacts, unsigned p_limit) const noexcept
	{
		if(isLeaf() || p_limit == 0) return 0;

		return children_[0]->getPotentialContactsWith(children_[1], p_contacts, p_limit);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename BoundingVolume>
	void
	BVHNode<BoundingVolume>::insert(const BoundingVolume& p_newVolume, RigidBody* p_newBody) noexcept
	{
		if(isLeaf())
		{
			//Child 0 is a copy of us
			children_[0] = std::make_unique<BVHNode<BoundingVolume>>(this, volume_, body_);

			//Child 1 holds the new body
			children_[1] = std::make_unique<BVHNode<BoundingVolume>>(this, p_newVolume, p_newBody);

			//We're no longer a leaf
			this->body_ = nullptr;

			recalculateBoundingVolume();
		}
		else //If we're not a leaf whe give the new body to the child who would grow the least
		{
			float growth_child0 = children_[0]->volume_.getGrowth(p_newVolume);
			float growth_child1 = children_[1]->volume_.getGrowth(p_newVolume);

			if(growth_child0 < growth_child1)
				children_[0]->insert(p_newVolume, p_newBody);
			else
				children_[1]->insert(p_newVolume, p_newBody);
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename BoundingVolume>
	bool
	BVHNode<BoundingVolume>::overlaps(const BVHNode<BoundingVolume>* p_other) const noexcept
	{
		return volume_.overlaps(p_other->volume_);
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	
	template <typename BoundingVolume>
	unsigned
	BVHNode<BoundingVolume>::getPotentialContactsWith(const BVHNode<BoundingVolume>* p_other,
		PotentialContact* p_contacts, unsigned p_limit) const noexcept
	{
		if(!overlaps(p_other) || p_limit == 0) return 0;

		if(isLeaf() && p_other->isLeaf())
		{
			p_contacts->bodies_[0] = body_;
			p_contacts->bodies_[1] = p_other->body_;

			return 1;
		}

		if(p_other->isLeaf() || 
			(!isLeaf() && volume_.getSize() >= p_other->volume_.getSize()))
		{
			//Recurse into ourself
			unsigned count = children_[0]->getPotentialContactsWith(p_other, p_contacts, p_limit);

			if(p_limit > count)
				return count + children_[1]->getPotentialContactsWith(p_other, p_contacts + count, p_limit - count);
			else
				return count;
		}
		else
		{
			//Recurse into the other node
			unsigned count = getPotentialContactsWith(p_other->children_[0], p_contacts, p_limit);

			if(p_limit > count)
				return count + getPotentialContactsWith(p_other->children_[1], p_contacts + count, p_limit - count);
			else
				return count;
		}
	}

	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------

	template <typename BoundingVolume>
	void
	BVHNode<BoundingVolume>::recalculateBoundingVolume(bool p_recurse) noexcept
	{
		if(isLeaf()) return;

		volume_ = BoundingVolume(children_[0]->volume_, children_[1]->volume_);

		//Recurse up the tree
		if(parent_) parent_->recalculateBoundingVolume(true);
	}
}//namespace Ocacho::Physics::RigidBody