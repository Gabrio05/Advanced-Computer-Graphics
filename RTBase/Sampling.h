#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	// 0.999999 to reduce noise on BoxFilter (removes floating point error which gives the sample to the next pixel instead)
	// (1.0f can't actually be generated anyway, but later operations with the generated result can)
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		return SphericalCoordinates::sphericalToWorld(acosf(r1), 2 * M_PI * r2);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		return M_1_PI / 2;
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		return SphericalCoordinates::sphericalToWorld(acosf(sqrtf(r1)), 2 * M_PI * r2);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		return cosf(wi.z) * M_1_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		return SphericalCoordinates::sphericalToWorld(acosf(1 - 2 * r1), 2 * M_PI * r2);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		return M_1_PI / 4;
	}
};