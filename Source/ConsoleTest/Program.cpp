#include "Particle.h"
#include <iostream>
#include <stdio.h>
using namespace std;
using namespace PolishPhysics;

int main()
{
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

	getchar();
}