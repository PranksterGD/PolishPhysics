#include "pch.h"
#include <windows.h>
#include "glut.h"
#include <glut.h>
#include <GL/gl.h>
#include "Particle.h"
#include "Projectile.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
using namespace std;
using namespace PolishPhysics;

float angle = 0.0f;
RigidBodyWorld world;

RigidBody body(0.0f);
RigidBody otherBody(0.0f);

BoundingSphere sphere(Vector3(0.0f, 0.0f, -10.0f), 1.414f);
BVHNode<BoundingSphere> volume(nullptr, sphere, &body);

CollisionBox box;
CollisionBox otherBox;

CollisionSphere csphere;
CollisionSphere otherCsphere;

BoundingSphere OtherSphere(Vector3(2.0f, 5.0f, -10.0f), 1.414f);
BVHNode<BoundingSphere> OtherVolume(nullptr, OtherSphere, &otherBody);

Contact contacts[10];
CollisionData collisionData;

CollisionDetector detector;


void ProcessKeys(unsigned char key, int x, int y)
{
	switch (key)
	{
	case ' ':
		otherBody.SetVelocity(Vector3(0.0f, 0.0f, 0.0f));
		break;
	case  'a':
		body.SetRotation(Vector3(0.0f, 10.0f, 0.0f));
		break;

	case 'd':
		body.SetRotation(Vector3(0.0f, -10.0f, 0.0f));
		break;

	case 'w':
		body.SetRotation(Vector3(-10.0f, 00.0f, 0.0f));
		break;

	case  's':
		body.SetRotation(Vector3(10.0f, 00.0f, 0.0f));
		break;

	case  'q':
		body.SetRotation(Vector3(0.0f, 00.0f, 10.0f));
		break;

	case  'e':
		body.SetRotation(Vector3(0.0f, 00.0f, -10.0f));
		break;

	case  'z':
		body.SetRotation(Vector3(0.0f, 00.0f, 0.0f));
		break;
	}

}

void ProcessSpecialKeys(int key, int x, int y)
{
	switch (key)
	{
	case  GLUT_KEY_UP:
		otherBody.SetVelocity(Vector3(0.0f, 0.5f, 0.0f));
		break;

	case GLUT_KEY_DOWN:
		otherBody.SetVelocity(Vector3(0.0f, -0.5f, 0.0f));
		break;

	case GLUT_KEY_LEFT:
		otherBody.SetVelocity(Vector3(-0.5f, 0.0f, 0.0f));
		break;

	case  GLUT_KEY_RIGHT:
		otherBody.SetVelocity(Vector3(0.5f, 0.0f, 0.0f));
		break;

	case  GLUT_KEY_HOME:
		otherBody.SetVelocity(Vector3(0.0f, 0.0f, 0.5f));
		break;

	case  GLUT_KEY_END:
		otherBody.SetVelocity(Vector3(0.0f, 0.0f, -0.5f));
		break;
	}
}

void RenderScene()
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glPushMatrix();
	//glPopMatrix();

	//glutSwapBuffers();
}

void Update()
{
	OtherSphere.SetCenter(otherBody.GetPosition());

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (CollisionDetector::SphereAndSphere(csphere, otherCsphere, &collisionData) > 0)
	{
		glColor3f(1, 0, 0);
	}
	else if (sphere.Overlaps(OtherSphere))
	{
		glColor3f(1, 1, 0);
	}
	else
	{
		glColor3f(1, 1, 1);
	}

	glPushMatrix();
	glTranslatef(body.GetPosition().X, body.GetPosition().Y, body.GetPosition().Z);
	glutSolidSphere(1,100,100);
//	glutSolidCube(1);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(otherBody.GetPosition().X, otherBody.GetPosition().Y, otherBody.GetPosition().Z);

	Quaternion rotation = otherBody.GetOrientation();

	float angle = 2 * acos(rotation.r);
	float x = rotation.i / sin(angle / 2);
	float y = rotation.j / sin(angle / 2);
	float z = rotation.k / sin(angle / 2);
	angle *= 57.2958f;

	//glScalef(1.0, 1.0f, 1.0f);
	glRotatef(angle, x, y, z);
	glutSolidSphere(1, 100, 100);
//	glutSolidCube(1);
	glPopMatrix();

	glutSwapBuffers();

	world.StartFrame();

	//body.AddForceAtPoint(Vector3(100.0f, 0.0f, 0.0f), Vector3(10.0f, 10.f, 10.0f));

	world.Update((float)1 / 60);

	Vector3 position = OtherSphere.mCenter;
	//cout << position.X << " " << position.Y << " " << position.Z << "\n";

	csphere.CalculateInternals();
	otherCsphere.CalculateInternals();

	box.CalculateInternals();
	otherBox.CalculateInternals();

	collisionData.Reset(2);
}

void ChangeSize(int width, int height)
{
	if (height == 0)
	{
		height = 1;
	}

	float ratio = 1.0f* width / height;

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	glViewport(0, 0, width, height);

	gluPerspective(45, ratio, 1, 1000);

	glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv)
{
	//init GLUT and create window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(320, 320);
	glutCreateWindow("Blah");

	gluLookAt(0.0f, 0.0f, 10.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f);


	//register callbacks
	glutDisplayFunc(RenderScene);
	glutReshapeFunc(ChangeSize);
	glutIdleFunc(Update);

	//Register input
	glutKeyboardFunc(ProcessKeys);
	glutSpecialFunc(ProcessSpecialKeys);

	body.SetMass(10.0f);
	Matrix3 intertiaTensor;
	intertiaTensor.mData[0] = 1.0f * 10 / 6;
	intertiaTensor.mData[4] = 1.0f * 10 / 6;
	intertiaTensor.mData[8] = 1.0f * 10 / 6;

	body.SetIntertiaTensor(intertiaTensor);
	body.SetPosition(Vector3(0.0, 0.0f, -10.0f));
	world.AddBody(body);

	box.halfSize = Vector3(0.5, 0.5, 0.5);
	box.mBody = &body;

	csphere.mRadius = 1.0f;
	csphere.mBody = &body;

	otherBody.SetMass(10.0f);
	otherBody.SetIntertiaTensor(intertiaTensor);
	otherBody.SetPosition(Vector3(2.0, 5.0f, -10.0f));
	world.AddBody(otherBody);

	otherBox.halfSize = Vector3(5, 5, 5);
	otherBox.mBody = &otherBody;

	otherCsphere.mRadius = 1.0f;
	otherCsphere.mBody = &otherBody;

	collisionData.mContacts = contacts;
	collisionData.mFirstContact = contacts;
	collisionData.mContactsLeft = 2;
	collisionData.mContactsFound = 0;
	collisionData.mFriction = 0.0f;
	collisionData.mRestitution = 0.0f;

	//enter GLUT event processing cycle
	glutMainLoop();

	return 0;
}
