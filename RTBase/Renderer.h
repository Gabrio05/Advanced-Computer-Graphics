#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular() == true) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (light->isArea()) {
			float pdf;
			Colour colour;
			Vec3 light_point = light->sample(shadingData, sampler, colour, pdf);
			Vec3 direction = (light_point - shadingData.x).normalize();
			float geo = std::max(Dot(direction, shadingData.sNormal), 0.0f) * std::max(Dot(-direction, light->normal(shadingData, direction)), 0.0f) / ((light_point - shadingData.x) * (light_point - shadingData.x)).length();
			if (geo > 0 && scene->visible(shadingData.x, light_point)) {
				Colour colour2 = shadingData.bsdf->evaluate(shadingData, direction);
				return Colour(geo, geo, geo) * colour * colour2 / (pmf * pdf);
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		for (int x = 0; x < 1; x++) {
			film->incrementSPP();
			for (int y = 0; y < film->height; y++) {
				for (int x = 0; x < film->width; x++) {
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);
					//Colour col = viewNormals(ray);
					//Colour col = albedo(ray);
					Colour col = direct(ray, samplers);
					film->splat(px, py, col);
				}
			}
		}
		for (int y = 0; y < film->height; y++) {
			for (int x = 0; x < film->width; x++) {
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
		film->clear();
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};