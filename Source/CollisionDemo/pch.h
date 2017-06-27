#pragma once

//Standard Headers
#include <math.h>
#include <assert.h>

//Local headers
#include "Particle.h"
#include "Projectile.h"
#include "ParticleContact.h"
#include "ParticleContactResolver.h"
#include "ParticleWorld.h"
#include "RigidBody.h"
#include "RigidBodyWorld.h"

//Math
#include "Precision.h"
#include "Vector3.h"
#include "Matrix3.h"
#include "Matrix4.h"
#include "Quaternion.h"

//ForceGenerators
#include "ParticleForceGenerator.h"
#include "ParticleForceRegistry.h"
#include "ParticleGravityForceGenerator.h"
#include "ParticleDragForceGenerator.h"
#include "ParticleSpringForceGenerator.h"
#include "ParticleAnchoredSpringForceGenerator.h"
#include "ParticleBungeeForceGenerator.h"
#include "ParticleAnchoredBungeeForceGenerator.h"
#include "ParticleBuoyancyForceGenerator.h"
#include "ParticleStiffSpringForceGenerator.h"

#include "RigidBodyForceGenerator.h"
#include "RigidyBodyGravityForceGenerator.h"
#include "RigidBodySpringForceGenerator.h"
#include "RigidBodyForceRegistry.h"

//Contact Generators
#include "ParticleContactGenerator.h"
#include "ParticleLink.h"
#include "ParticleCable.h"
#include "ParticleRod.h"


//Collision Detection
#include "BoundingSphere.h"
#include "BVHNode.h"
#include "PotentialContact.h"
#include "Primitive.h"
#include "Contact.h"
#include "CollisionData.h"
#include "CollisionDetector.h"
#include "IntersectionTests.h"

#include "Sphere.h"
#include "Plane.h"
#include "Box.h"
