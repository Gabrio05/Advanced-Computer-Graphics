#include "pch.h"
#include "CppUnitTest.h"
#include "..\RTBase\Geometry.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTest1
{
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(TestMethod1)
		{
			Ray ray{Vec3(0, 0, 0), Vec3(0, -1, 0)};
			Plane plane{};
			plane.init(Vec3(0, 1, 0), 5);
			float time = 0.0f;
			Assert::AreEqual(plane.rayIntersect(ray, time), true);
			Assert::AreEqual(time, 5.0f);
		}

		TEST_METHOD(TestMethod2) {
			Ray ray{ Vec3(0, 1, 0), Vec3(0, -1, 0) };
			Triangle triangle{};
			triangle.init(Vertex{Vec3(-1, 0, -1), Vec3(0, 0, 0), 0, 0}, Vertex{ Vec3(-1, 0, 1), Vec3(0, 0, 0), 0, 0 }, Vertex{ Vec3(1, 1, 0), Vec3(0, 0, 0), 0, 0 }, 0);
			float time = 0.0f;
			float u = 0.0f;
			float v = 0.0f;
			Assert::AreEqual(triangle.rayIntersect(ray, time, u, v), true);
			//Assert::AreEqual(time, 1.0f);
		}
	};
}
