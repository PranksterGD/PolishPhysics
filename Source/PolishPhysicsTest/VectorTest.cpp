#include "pch.h"
#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace PolishPhysics;
using namespace std;

namespace PolishPhysicsTest
{		
	TEST_CLASS(VectorTest)
	{
	public:

		TEST_METHOD_INITIALIZE(Initialize)
		{
#if defined (_DEBUG)

			_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF);
			_CrtMemCheckpoint(&sStartMemState);
#endif
		}

		TEST_METHOD_CLEANUP(Cleanup)
		{
#if defined (_DEBUG)
			_CrtMemState endMemState, diffMemState;
			_CrtMemCheckpoint(&endMemState);
			if (_CrtMemDifference(&diffMemState, &sStartMemState, &endMemState))
			{
				_CrtMemDumpStatistics(&diffMemState);
				Assert::Fail(L"Memory Leaks!");
			}
#endif
		}
		
		TEST_METHOD(DefaultConstructorTest)
		{
			Vector3 a;

			Assert::IsTrue(a.X == 0.0f);
			Assert::IsTrue(a.X == 0.0f);
			Assert::IsTrue(a.X == 0.0f);
		}

		TEST_METHOD(CopyConstructorTest)
		{
			Vector3 a;
			a.X = 10;
			a.Y = 20;
			a.Z = 30;

			Vector3 b(a);

			Assert::IsTrue(b.X == 10);
			Assert::IsTrue(b.Y == 20);
			Assert::IsTrue(b.Z == 30);
		}

		TEST_METHOD(MoveConstructorTest)
		{
			Vector3 a;
			a.X = 10;
			a.Y = 20;
			a.Z = 30;

			Vector3 b(move(a));

			Assert::IsTrue(b.X == 10);
			Assert::IsTrue(b.Y == 20);
			Assert::IsTrue(b.Z == 30);

			Assert::IsTrue(a.X == 0.0f);
			Assert::IsTrue(a.X == 0.0f);
			Assert::IsTrue(a.X == 0.0f);
		}

		TEST_METHOD(AssignmentOperatorTest)
		{
			Vector3 a;
			a.X = 10;
			a.Y = 20;
			a.Z = 30;

			Vector3 b;

			b = a;

			Assert::IsTrue(b.X == 10);
			Assert::IsTrue(b.Y == 20);
			Assert::IsTrue(b.Z == 30);
		}

		TEST_METHOD(MoveAssignmentOperatorTest)
		{
			Vector3 a;
			a.X = 10;
			a.Y = 20;
			a.Z = 30;

			Vector3 b;
			b = move(a);

			Assert::IsTrue(b.X == 10);
			Assert::IsTrue(b.Y == 20);
			Assert::IsTrue(b.Z == 30);

			Assert::IsTrue(a.X == 0.0f);
			Assert::IsTrue(a.X == 0.0f);
			Assert::IsTrue(a.X == 0.0f);
		}

		TEST_METHOD(EqualityOperatorTest)
		{
			Vector3 a(10, 20, 30);
			Vector3 b(10, 20, 30);

			Assert::IsTrue(a == b);

			Vector3 c(10, 20, 0);

			Assert::IsFalse(a == c);
		}

		TEST_METHOD(InequalityOperatorTest)
		{
			Vector3 a(10, 20, 30);
			Vector3 b(10, 20, 30);

			Assert::IsFalse(a != b);

			Vector3 c(10, 20, 0);

			Assert::IsTrue(a != c);
		}

		TEST_METHOD(InvertTest)
		{
			Vector3 a(10, 20, 30);
			Vector3 b(-10, -20, -30);

			a.Invert();

			Assert::IsTrue(a == b);
		}

		TEST_METHOD(MagnitudeTest)
		{
			Vector3 a(1, 1, 1);

			Vector3 b(2, 3, 5);

			Assert::AreEqual(a.Magnitude(), sqrtf(3));
			Assert::AreEqual(b.Magnitude(), sqrtf(38));
		}

		TEST_METHOD(SquareMagnitudeTest)
		{
			Vector3 a(1, 1, 1);

			Vector3 b(2, 3, 5);

			Assert::AreEqual(a.SquareMagnitude(), 3.0f);
			Assert::AreEqual(b.SquareMagnitude(),38.0f);
		}

		TEST_METHOD(NormalizeTest)
		{
			Vector3 a(1, 1, 1);

			a.Normalize();

			Assert::IsTrue(a == Vector3(1 / sqrtf(3), 1 / sqrtf(3), 1 / sqrtf(3)));

			Vector3 b(2, 3, 5);

			b.Normalize();

			Assert::AreEqual(b.X, 2 / sqrtf(38));
			Assert::AreEqual(b.Z, 5 / sqrtf(38));
		}

		TEST_METHOD(ScalarMultiplyAssignmentTest)
		{
			Vector3 a(2, 3, 5);

			a *= 2;

			Assert::IsTrue(a == Vector3(4, 6, 10));
		}

		TEST_METHOD(ScalarMultplicationTest)
		{
			Vector3 a(2, 3, 5);

			Vector3 b = a * 2;

			Assert::IsTrue(b == Vector3(4, 6, 10));
		}

		TEST_METHOD(AdditionAssignmentTest)
		{
			Vector3 a(2, 3, 5);

			Vector3 b(1, 1, 1);

			a += b;

			Assert::IsTrue(a == Vector3(3, 4, 6));
		}

		TEST_METHOD(AdditionTest)
		{
			Vector3 a(2, 3, 5);

			Vector3 b(1, 1, 1);

			Vector3 c = a + b;

			Assert::IsTrue(c == Vector3(3, 4, 6));
		}

		TEST_METHOD(SubtractionAssignmentTest)
		{
			Vector3 a(2, 3, 5);

			Vector3 b(1, 1, 1);

			a -= b;

			Assert::IsTrue(a == Vector3(1, 2, 4));
		}

		TEST_METHOD(SubtractionTest)
		{
			Vector3 a(2, 3, 5);

			Vector3 b(1, 1, 1);

			Vector3 c = a - b;

			Assert::IsTrue(c == Vector3(1, 2, 4));
		}

		TEST_METHOD(AddScaledVectorTest)
		{
			Vector3 a(2, 3, 5);

			Vector3 b(1, 1, 1);

			a.AddScaledVector(b, 2);

			Assert::IsTrue(a == Vector3(4, 5, 7));
		}

		TEST_METHOD(ComponentMultiplcationAssignment)
		{
			Vector3 a(2,3,5);
			Vector3 b(5, 4, 2);

			a.ComponentProductAssignment(b);

			Assert::IsTrue(a == Vector3(10, 12, 10));
		}

		TEST_METHOD(ComponentMultiplcation)
		{
			Vector3 a(2, 3, 5);
			Vector3 b(5, 4, 2);

			Vector3 c = a.ComponentProduct(b);

			Assert::IsTrue(c == Vector3(10, 12, 10));
		}

		TEST_METHOD(ScalarProductTest)
		{
			Vector3 a(2, 3, 5);
			Vector3 b(5, 4, 2);

			Precision c = a.ScalarProduct(b);

			Assert::IsTrue(c == 32);
		}

		TEST_METHOD(VectorProductTest)
		{
			Vector3 a(2, 3, 5);
			Vector3 b(5, 4, 2);

			Vector3 c = a.VectorProduct(b);

			Assert::IsTrue(c == Vector3(-14, 21, -7));
		}

		TEST_METHOD(VectorProductAssignmentTest)
		{
			Vector3 a(2, 3, 5);
			Vector3 b(5, 4, 2);

			a.VectorProductAssignment(b);

			Assert::IsTrue(a == Vector3(-14, 21, -7));
		}

		TEST_METHOD(OrthoNormalBasisTest)
		{
			Vector3 a(2, 3, 5);
			Vector3 b(4, 6, 6);
			Vector3 c(2, 7, 5);

			Vector3::MakeOrthonormalBasis(a, b, c);

			Assert::AreEqual(a.ScalarProduct(b), 0.0f);
			Assert::AreEqual(a.ScalarProduct(c), 0.0f);
			Assert::AreEqual(b.ScalarProduct(c), 0.0f);

			Vector3 d(1, 1, 1);
			Vector3 e(2, 2, 2);

			Assert::IsFalse(Vector3::MakeOrthonormalBasis(d, e, c));
		}

		TEST_METHOD(ZeroVectorTest)
		{
			Assert::IsTrue(Vector3::ZeroVector() == Vector3(0, 0, 0));
		}

	private:
		static _CrtMemState sStartMemState;

	};
	_CrtMemState VectorTest::sStartMemState;

}