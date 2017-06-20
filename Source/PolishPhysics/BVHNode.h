#pragma once
#include <cstdint>
#include "RigidBody.h"
#include "PotentialContact.h"


namespace PolishPhysics
{
	template <class BoundingVolumeClass>
	class BVHNode
	{
		private:

		// Holds the child nodes of this node
		BVHNode* mchildren[2];

		//Holds a single bounding volume encompassing all the descendants of this node.
		BoundingVolumeClass mVolume;

		//Holds the rigid body at this node. Only leaf nodes can have a rigid body.
		RigidBody* mBody;

		//Holds the parent up the hierarchy.
		BVHNode* mParent;

		BVHNode(BVHNode *parent, const BoundingVolumeClass &volume, RigidBody* body = nullptr):
			mParent(parent), mVolume(volume), mBody(body)
		{
			mchildren[0] = mchildren[1] = nullptr;
		}

		~BVHNode();

	public:

		/**Checks if this node is at the bottom of the hierarchy. */
		bool IsLeaf() const;

		/**Checks the potential contacts from this node downwards in the hierarchy.
		* @param contacts- The array of contacts to write to.
		* @param limit - The maximum number of contacts that can be written.
		* @return - The number of contacts written.*/
		std::uint32_t GetPotentialContacts(PotentialContact* contacts, std::uint32_t limit) const;

		/**Checks the potential contacts between this node and another node.
		* @param other - The node to check against.
		* @param contacts- The array of contacts to write to,
		* @param limit - The maximum number of contacts that can be written.
		* @return - The number of contacts written.*/
		std::uint32_t GetPotentialContactsWith(const BVHNode<BoundingVolumeClass>& other,
			PotentialContact* contacts, std::uint32_t limit) const;

		/**Checks if this node overlaps with another node.
		* @param other - The node to check against.*/
		bool Overlaps(const BVHNode<BoundingVolumeClass>& other) const;

		/**Inserts the given rigid body, with the given bounding volume, into the hierarchy
		* @param body - The new rigid body to add.
		* @param volume - The bounding volume attached to the body.*/
		void Insert(RigidBody& newBody, const BoundingVolumeClass& newVolume);

		/**For non-leaf nodes, this method recalculates the bounding volume
		* based on the bounding volumes of its children.*/
		void RecalculateBoundingVolume(bool recurse = true);
	};
}

#include "BVHNode.inl"