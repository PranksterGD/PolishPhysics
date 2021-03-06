#pragma once
namespace PolishPhysics
{
	template<class BoundingVolumeClass>
	bool BVHNode<BoundingVolumeClass>::IsLeaf() const
	{
		return mBody != nullptr;
	}

	template<class BoundingVolumeClass>
	bool BVHNode<BoundingVolumeClass>::Overlaps(const BVHNode<BoundingVolumeClass>& other) const
	{
		return mVolume.Overlaps(other.mVolume);
	}

	template<class BoundingVolumeClass>
	std::uint32_t BVHNode<BoundingVolumeClass>::GetPotentialContacts(PotentialContact* contacts, std::uint32_t limit) const
	{
		std::uint32_t contactsGenerated;

		if (IsLeaf() || limit == 0)
		{
			contactsGenerated = 0;
		}
		else
		{
			contactsGenerated = mChildren[0]->GetPotentialContactsWith(mChildren[1], contacts, limit);
		}

		return contactsGenerated;
	}

	template<class BoundingVolumeClass>
	std::uint32_t BVHNode<BoundingVolumeClass>::GetPotentialContactsWith(const BVHNode<BoundingVolumeClass>& other, PotentialContact* contacts, std::uint32_t limit) const
	{
		//Early out if we don't overlap or if we cannot add more contacts.
		if (!Overlaps(other) || limit == 0)
		{
			return 0;
		}

		//If both are leaf nodes, then there is exactly one potential contact.
		if (IsLeaf() && other.IsLeaf())
		{
			contacts->mbodies[0] = mBody;
			contacts->mbodies[1] = other.mBody;
			return 1;
		}

		//Determine which node to descend into. If either is a leaf, we descend the other.
		//If both are branches, we descend down the one that is bigger.
		if (other.IsLeaf() || (!IsLeaf() && mVolume.GetSize() >= other.mVolume.GetSize()))
		{
			std::uint32_t contactsGenerated = mChildren[0]->GetPotentialContactsWith(other, contacts, limit);

			if (limit > contactsGenerated)
			{
				return contactsGenerated + mChildren[1]->GetPotentialContactsWith(other, contacts + contactsGenerated, limit - contactsGenerated);
			}
			else
			{
				return contactsGenerated;
			}
		}

		//Recurse into the other node.
		else
		{
			std::uint32_t contactsGenerated = GetPotentialContactsWith(other.mChildren[0], contacts, limit);

			if (limit > contactsGenerated)
			{
				return contactsGenerated + GetPotentialContactsWith(other.mChildren[1], contacts + contactsGenerated, limit - contactsGenerated);
			}
			else
			{
				return contactsGenerated;
			}
		}
	}

	template<class BoundingVolumeClass>
	void BVHNode<BoundingVolumeClass>::Insert(RigidBody& newBody, const BoundingVolumeClass& newVolume)
	{
		//If we are a leaf then the only option it to spawn two new children.

		if (IsLeaf())
		{
			//Child one is a copy of us.
			mChildren[0] = new BVHNode<BoundingVolumeClass>(this, mVolume, mBody);
			mChildren[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);

			mBody = nullptr;

			RecalculateBoundingVolume();
		}
		//Otherwise, work out which child gets the inserted body. Give it to whichever would grow the least
		else
		{
			if (mChildren[0]->mVolume.GetGrowth(newVolume) < mChildren[1]->mVolume.GetGrowth(newVolume))
			{
				mChildren[0]->Insert(newBody, newVolume);
			}
			else
			{
				mChildren[1]->Insert(newBody, newVolume);
			}
		}
	}

	template<class BoundingVolumeClass>
	BVHNode<BoundingVolumeClass>::~BVHNode()
	{
		//If we don't have a parent, then we ignore the sibling processing.

		if (mParent != nullptr)
		{
			//Find the sibling.

			BVHNode<BoundingVolumeClass>* sibling;
			
			if (mParent->mChildren[0] == this)
			{
				sibling = mParent->mChildren[1];
			}
			else
			{
				sibling = mParent->mChildren[0];
			}

			//Write its data to the parent.
			mParent->mVolume = sibling->mVolume;
			mParent->mBody = sibling->mBody;
			mParent->mChildren[0] = sibling->mChildren[0];
			mParent->mChildren[1] = sibling->mChildren[1];

			//Delete the sibling
			sibling->mParent = nullptr;
			sibling->mBody = nullptr;
			sibling->mChildren[0] = nullptr;
			sibling->mChildren[1] = nullptr;
			delete sibling;

			//Recalculate the parent's bounding volume.
			mParent->RecalculateBoundingVolume();
		}

		//Delete the children.
		if (mChildren[0] != nullptr)
		{
			mChildren[0]->mParent = nullptr;
			delete mChildren[0];
		}

		if (mChildren[1] != nullptr)
		{
			mChildren[1]->mParent = nullptr;
			delete mChildren[1];
		}
	}

	template<class BoundingVolumeClass>
	void BVHNode<BoundingVolumeClass>::RecalculateBoundingVolume(bool recurse /* = true */)
	{
		if (!IsLeaf())
		{
			// Use the bounding volume combining constructor.
			mVolume = BoundingVolumeClass(mChildren[0]->mVolume, mChildren[1]->mVolume);

			// Recurse up the tree
			if (mParent != nullptr)
			{
				mParent->RecalculateBoundingVolume(true);
			}
		}


	}
}

