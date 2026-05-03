#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	std::vector<float> row_cdf{};
	std::vector<float> individual_row_total_luminance{};
	float overall_luminance = 0.0f;
	float real_overall_luminance = 0.0f;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		row_cdf.reserve(env->height);
		float total_luminance = 0.0f;
		float real_total_luminance = 0.0f;
		for (int y = 0; y < env->height; y++) {
			float st = sinf(((float)y / (float)env->height) * M_PI);
			const float old_total_luminance = total_luminance;
			for (int x = 0; x < env->width; x++) {
				total_luminance += env->texels[y * env->width + x].Lum() * st;
				real_total_luminance += env->texels[y * env->width + x].Lum();
			}
			row_cdf.emplace_back(total_luminance);
			individual_row_total_luminance.emplace_back(total_luminance - old_total_luminance);
		}
		for (int y = 0; y < env->height; y++) {
			row_cdf.at(y) /= total_luminance;
		}
		overall_luminance = total_luminance;
		real_overall_luminance = real_total_luminance;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		auto it = std::lower_bound(row_cdf.begin(), row_cdf.end(), sampler->next());
		int height = it - row_cdf.begin();
		int width = 0;
		float st = sinf(((float)height / (float)env->height) * M_PI);
		float x_sample = sampler->next() * individual_row_total_luminance[height];
		for (int x = 0; x < env->width; x++) {
			x_sample -= env->texels[height * env->width + x].Lum() * st;
			if (x_sample < 0) {
				width = x;
				break;
			}
		}
		float theta = (float)height / (float)env->height * M_PI;
		float phi = 2 * M_PI * (float)width / (float)env->width;
		Vec3 wi = Vec3(cosf(phi) * sinf(theta), cosf(theta), sinf(phi) * sinf(theta));
		//wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		//pdf = SamplingDistributions::uniformSpherePDF(wi);
		pdf = 1.0f * (env->width * env->height) / (2 * M_PI * M_PI * sinf(theta));
		//pdf = env->texels[height * env->width + width].Lum() * (env->width * env->height) / (real_overall_luminance * 2 * M_PI * M_PI);
		reflectedColour = evaluate(wi);
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		//return SamplingDistributions::uniformSpherePDF(wi);
		return 1.0f * (env->width * env->height) / (2 * M_PI * M_PI * sinf(acosf(wi.y)));
		//return evaluate(wi).Lum() * (env->width * env->height) / (real_overall_luminance * 2 * M_PI * M_PI);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return overall_luminance * 4.0f * M_PI;
		//float total = 0;
		//for (int i = 0; i < env->height; i++)
		//{
		//	float st = sinf(((float)i / (float)env->height) * M_PI);
		//	for (int n = 0; n < env->width; n++)
		//	{
		//		total += (env->texels[(i * env->width) + n].Lum() * st);
		//	}
		//}
		//total = total / (float)(env->width * env->height);
		//return total * 4.0f * M_PI;
	}
	// Oops, didn't see these last 2 functions, I think I manually implemented them directly
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class VirtualPointLight : public Light {
public:
	Vec3 position{};
	Vec3 normal_vector{};
	Colour emission{};
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) {
		emittedColour = emission;
		pdf = 1.0f;
		return position;
	}
	Colour evaluate(const Vec3& wi) {
		if (Dot(wi, normal_vector) < 0) {
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		return 0.0f;
	}
	bool isArea() {
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi) {
		return normal_vector;
	}
	float totalIntegratedPower() {
		return emission.Lum();
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) {
		return position;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) {
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(normal_vector);
		return frame.toWorld(wi);
	}
};