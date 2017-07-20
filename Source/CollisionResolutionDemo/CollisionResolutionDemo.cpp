#include "ProjectPolish.h"

#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

#include <stdio.h>

#include <iostream>


using namespace PolishPhysics;

class Block : public CollisionBox
{
public:
	bool exists;

	Block()
		:
		exists(false)
	{
		mBody = new RigidBody();
	}

	~Block()
	{
		delete mBody;
	}

	/** Draws the block. */
	void render()
	{
		// Get the OpenGL transformation
		GLfloat mat[16];
		mBody->GetGLTransform(mat);

		glPushMatrix();
		glMultMatrixf(mat);
		glScalef(halfSize.X * 2, halfSize.Y * 2, halfSize.Z * 2);
		glutSolidCube(1.0f);
		glPopMatrix();
	}

	/** Sets the block to a specific location. */
	void setState(const Vector3 &position,
		const Quaternion &orientation,
		const Vector3 &extents,
		const Vector3 &velocity)
	{
		mBody->SetPosition(position);
		mBody->SetOrientation(orientation);
		mBody->SetVelocity(velocity);
		mBody->SetRotation(Vector3(0, 0, 0));
		halfSize = extents;

		mBody->SetLinearDamping(0.95f);
		mBody->SetAngularDamping(0.8f);
		mBody->ClearAccumulators();
		mBody->SetAcceleration(Vector3(0, -10.0f, 0));

		//body->setCanSleep(false);
		mBody->SetAwake(true);

		mBody->CalculateDerivedData();
	}

	/**
	* Calculates and sets the mass and inertia tensor of this block,
	* assuming it has the given constant density.
	*/
	void calculateMassProperties(Precision invDensity)
	{
		// Check for infinite mass
		if (invDensity <= 0)
		{
			// Just set zeros for both mass and inertia tensor
			mBody->SetInverseMass(0);
			mBody->SetInverseIntertiaTensor(Matrix3());
		}
		else
		{
			// Otherwise we need to calculate the mass
			Precision volume = halfSize.Magnitude() * 2.0;
			Precision mass = volume / invDensity;

			mBody->SetMass(10);

			// And calculate the inertia tensor from the mass and size
			mass *= 0.333f;
			Matrix3 tensor;
			tensor.SetInertiaTensorCoeffs(
				mass * halfSize.Y*halfSize.Y + halfSize.Z*halfSize.Z,
				mass * halfSize.Y*halfSize.X + halfSize.Z*halfSize.Z,
				mass * halfSize.Y*halfSize.X + halfSize.Z*halfSize.Y
			);
			mBody->SetIntertiaTensor(tensor);
		}

	}
};

/**
* The main demo class definition.
*/
class FractureDemo : public RigidBodyApplication
{
	/** Tracks if a block has been hit. */

	unsigned contact;

	/** Holds the bodies. */
	Block block;

	/** Holds the projectile. */
	CollisionSphere ball;

	/** Processes the contact generation code. */
	virtual void generateContacts();

	/** Processes the objects in the simulation forward in time. */
	virtual void updateObjects(Precision duration);

	/** Resets the position of all the blocks. */
	virtual void reset();

	/** Processes the physics. */
	virtual void update();

	virtual void key(unsigned char key) override;

public:
	/** Creates a new demo object. */
	FractureDemo();

	/** Returns the window title for the demo. */
	virtual const char* getTitle();

	/** Display the particle positions. */
	virtual void display();
};

// Method definitions
FractureDemo::FractureDemo()
	:
	RigidBodyApplication()
{
	// Create the ball.
	ball.mBody = new RigidBody();
	ball.mRadius = 2;
	ball.mBody->SetMass(5.0f);
	Matrix3 it;
	it.SetDiagonal(5.0f, 5.0f, 5.0f);
	ball.mBody->SetIntertiaTensor(it);
	ball.mBody->SetAcceleration(Vector3(0, -10,0));

//	ball.mBody->setCanSleep(false);
	ball.mBody->SetAwake(true);

	// Set up the initial block
	reset();
}

const char* FractureDemo::getTitle()
{
	return "Collision Resolution";
}

void FractureDemo::generateContacts()
{

	// Create the ground plane data
	CollisionPlane plane;
	plane.mNormal =Vector3(0, 1, 0);
	plane.mOffset = 0;
	plane.mHasOffset = false;

	// Set up the collision data structure
	cData.Reset(maxContacts);
	cData.mFriction = (Precision)0.9;
	cData.mRestitution = (Precision)0.2;
	cData.mTolerance = (Precision)0.1;

	// Perform collision detection
	Matrix4 transform, otherTransform;
	Vector3 position, otherPosition;
	

	// Check for collisions with the ground plane
	if (!cData.HasMoreContacts()) return;
	CollisionDetector::BoxAndHalfSpace(block, plane, &cData);

	
	// And with the sphere
	if (!cData.HasMoreContacts()) return;
	if (CollisionDetector::BoxAndSphere(block, ball, &cData))
	{
		contact = cData.mContactsLeft - 1;
	}
	
	

	// Check for sphere ground collisions
	if (!cData.HasMoreContacts()) return;
	CollisionDetector::SphereAndHalfSpace(ball, plane, &cData);
	
}

void FractureDemo::reset()
{

	// Set the first block
	block.halfSize = Vector3(4, 4, 4);
	block.mBody->SetPosition(Vector3(0, 7, 0));
	block.mBody->SetOrientation(Quaternion(1, 0, 0, 0));
	block.mBody->SetVelocity(Vector3(0, 0, 0));
	block.mBody->SetRotation(Vector3(0, 0, 0));
	block.mBody->SetMass(100.0f);
	Matrix3 it;
	it.SetBlockInertiaTensor(block.halfSize, 100.0f);
	block.mBody->SetIntertiaTensor(it);
	block.mBody->CalculateDerivedData();
	block.CalculateInternals();

	block.mBody->SetAcceleration(Vector3(0, -10, 0));
	block.mBody->SetAwake(true);

	// Set up the ball
	ball.mBody->SetPosition(Vector3(0, 5.0f, 20.0f));
	ball.mBody->SetOrientation(Quaternion(1, 0, 0, 0));
				
	ball.mBody->SetRotation(Vector3(0, 0, 0));
	ball.mBody->CalculateDerivedData();
	ball.mBody->SetAwake(true);
	ball.CalculateInternals();

	// Reset the contacts
	cData.mContactsFound = 0;
}

void FractureDemo::update()
{
	RigidBodyApplication::update();
}

void FractureDemo::key(unsigned char key)
{
	switch (key)
	{
	case 'W': case 'w':
		// Reset the simulation
		ball.mBody->SetVelocity(Vector3(0, 10, 0));
		break;

	case 'E': case 'e':
		// Toggle rendering of contacts
		ball.mBody->SetVelocity(Vector3(10, 0, 0));
		break;

	case 'Q': case 'q':
		// Toggle running the simulation
		ball.mBody->SetVelocity(Vector3(-10, 0, 0));
		break;

	case 'S': case 's':
		// Advance one frame
		ball.mBody->SetVelocity(Vector3(0, -10, 0));
		break;

	case 'D': case 'd':
		// Advance one frame
		ball.mBody->SetVelocity(Vector3(0, 0, -10));
		break;

	case 'A': case 'a':
		// Advance one frame
		ball.mBody->SetVelocity(Vector3(0, 0, 10));
		break;

	case 'G': case 'g':
		// Advance one frame
		ball.mBody->SetVelocity(Vector3(0, 0, 0));
		break;

	}
}

void FractureDemo::updateObjects(Precision duration)
{

	block.mBody->Integrate(duration);
	block.CalculateInternals();

	ball.mBody->Integrate(duration);
	ball.CalculateInternals();

}

void FractureDemo::display()
{
	const static GLfloat lightPosition[] = { 0.7f,1,0.4f,0 };

	RigidBodyApplication::display();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_NORMALIZE);
	block.render();
	glDisable(GL_NORMALIZE);

	glColor3f(0.4f, 0.7f, 0.4f);
	glPushMatrix();
	Vector3 pos = ball.mBody->GetPosition();
	glTranslatef(pos.X, pos.Y, pos.Z);
	glutSolidSphere(2, 16, 8);
	glPopMatrix();
	

	glDisable(GL_LIGHTING);
	glDisable(GL_COLOR_MATERIAL);

	RigidBodyApplication::drawDebug();
}

/**
* Called by the common demo framework to create an application
* object (with new) and return a pointer.
*/
Application* getApplication()
{
	return new FractureDemo();
}