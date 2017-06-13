#include "pch.h"
#include "Particle.h"
#include "Projectile.h"
#include <iostream>
#include <stdio.h>
using namespace std;
using namespace PolishPhysics;

int main()
{/*
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

	RigidBodyWorld world;

	RigidBody body;
	body.SetMass(10.0f);
	Matrix3 intertiaTensor;
	intertiaTensor.mData[0] = 0.5f;
	intertiaTensor.mData[4] = 0.5f;
	intertiaTensor.mData[8] = 0.5f;

	body.SetIntertiaTensor(intertiaTensor);

	body.AddForceAtPoint(Vector3(100.0f, 0.0f,0.0f), Vector3(10.0f, 10.f, 10.0f));

	body.SetPosition(Vector3(100.0f, 0.0f, 0.0f));

	world.AddBody(body);

	for (int i = 0; i < 100; ++i)
	{
		world.StartFrame();

		body.AddForceAtPoint(Vector3(100.0f, 0.0f, 0.0f), Vector3(10.0f, 10.f, 10.0f));

		world.Update((float)1/60);

		Vector3 position = body.GetPosition();
		cout << position.X << " " << position.Y << " " << position.Z << "\n";

		Quaternion orientation = body.GetOrientation();
		cout << orientation.r << " " << orientation.i << " " << orientation.j << " "<< orientation.k << "\n";
	}

	getchar();

}