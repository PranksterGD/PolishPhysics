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

void RenderScene()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	glTranslatef(0, 0.0f, -5);
	glScalef(1.0, 1.0f, 1.0f);
	glRotatef(45, 0, 45, 0);
	glutSolidCube(1.0f);
	glPopMatrix();

	glutSwapBuffers();
}

void Update()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	glTranslatef(body.GetPosition().X, body.GetPosition().Y, body.GetPosition().Z);

	Quaternion rotation = body.GetOrientation();

	float angle = 2 * acos(rotation.r);
	float x = rotation.i / sin(angle / 2);
	float y = rotation.j / sin(angle / 2);
	float z = rotation.k / sin(angle / 2);
	angle *= 57.2958;

	//glScalef(1.0, 1.0f, 1.0f);
	glRotatef(angle,x,y,z);
	glutSolidCube(1.0f);
	glPopMatrix();

	glutSwapBuffers();

	world.StartFrame();

	//body.AddForceAtPoint(Vector3(100.0f, 0.0f, 0.0f), Vector3(10.0f, 10.f, 10.0f));

	body.SetRotation(Vector3(10.0f, 0.0f, 10.0f));

	world.Update((float)1 / 60);

	Vector3 position = body.GetPosition();
	cout << position.X << " " << position.Y << " " << position.Z << "\n";

	Quaternion orientation = body.GetOrientation();
//	cout << orientation.r << " " << orientation.i << " " << orientation.j << " " << orientation.k << "\n";
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

	//register callbacks
	glutDisplayFunc(RenderScene);
	glutReshapeFunc(ChangeSize);
	glutIdleFunc(Update);

	body.SetMass(10.0f);
	Matrix3 intertiaTensor;
	intertiaTensor.mData[0] = 1.0f * 10/6;
	intertiaTensor.mData[4] = 1.0f * 10 / 6;
	intertiaTensor.mData[8] = 1.0f * 10 / 6;

	body.SetIntertiaTensor(intertiaTensor);

//	body.AddForceAtPoint(Vector3(100.0f, 0.0f, 0.0f), Vector3(10.0f, 10.f, 10.0f));

	body.SetPosition(Vector3(0.0, 0.0f, -10.0f));

	world.AddBody(body);


	//enter GLUT event processing cycle
	glutMainLoop();

	return 0;
}

void ConsoleStuff()
{
	/*
	Particle a;

	a.SetPosition(Vector3(0, 100, 0));
	a.SetAcceleration(Vector3(0, -10, 0));
	a.SetMass(10);

	for (int i = 0; i < 100; ++i)
	{
	Vector3 position = a.GetPosition();
	cout << position.X << " " << position.Y << " " << position.Z << "\n";
	a.Integrate(0.167f);
	}

	getchar();*/

	/*cout << "\n BULLET\n\n\n";
	Projectile a;
	a.SetType(Projectile::ShotType::PISTOL);

	a.Fire();

	for (int i = 0; i < 100; ++i)
	{
	a.Update();

	Vector3 position = a.GetPosition();
	cout << position.X << " " << position.Y << " " << position.Z << "\n";

	if (a.IsExpired())
	{
	break;
	}
	}

	cout << "\n CROSSBOW\n\n\n";
	Projectile b;
	b.SetType(Projectile::ShotType::CROSSBOW);

	b.Fire();

	for (int i = 0; i < 100; ++i)
	{
	b.Update();

	Vector3 position = b.GetPosition();
	cout << position.X << " " << position.Y << " " << position.Z << "\n";

	if (b.IsExpired())
	{
	break;
	}
	}

	getchar();*/

	
}