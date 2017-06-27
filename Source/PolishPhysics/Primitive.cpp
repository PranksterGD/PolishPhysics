#include "pch.h"

using namespace std;
using namespace PolishPhysics;

Primitive::Primitive() :
	mHasOffset(false)
{

}

Vector3 Primitive::GetAxis(uint32_t index) const
{
	return mTransform.GetAxisVector(index);
}

const Matrix4& Primitive::GetTransform() const
{
	return mTransform;
}

void Primitive::CalculateInternals()
{
	if (mHasOffset)
	{
		mTransform = mBody->GetTransform() * mOffset;
	}
	else
	{
		mTransform = mBody->GetTransform();
	}
}