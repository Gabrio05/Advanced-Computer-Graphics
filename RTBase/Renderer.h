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
		if (shadingData.bsdf->isPureSpecular()) {
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
		// Direct
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		Colour directLight = Colour(0, 0, 0);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				if (depth == 0 || shadingData.bsdf->isPureSpecular()) {
					return directLight = shadingData.bsdf->emit(shadingData, shadingData.wo) * pathThroughput;
				}
				return Colour(0, 0, 0);
			}
			directLight = computeDirect(shadingData, sampler);
		} else {
			return scene->background->evaluate(r.dir) * pathThroughput;
		}
		directLight = directLight * pathThroughput;
		// Indirect
		float epsilon = 0.0001f;
		float c = 0.0f;
		Colour reflectedColour{};
		float pdf = 0;
		Vec3 next_vector = shadingData.bsdf->sample(shadingData, sampler, reflectedColour, pdf);
		Ray next = Ray(r.at(intersection.t) + shadingData.gNormal * epsilon, next_vector);
		float the_cos = cosf(Dot(shadingData.gNormal, -r.dir));
		pathThroughput = pathThroughput * reflectedColour * the_cos / pdf;
		float rrp = std::min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() > rrp) {
			return directLight + Colour(c, c, c) * pathThroughput;
		}
		float qc = (1 - pathThroughput.Lum()) * c;
		return directLight + (pathTrace(next, pathThroughput, depth + 1, sampler) - Colour(qc, qc, qc)) / (rrp);
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
		//film->SPP = 16;
		//for (int y = 0; y < film->height; y++) {
		//	for (int x = 0; x < film->width; x++) {
		//		for (int i = 0; i < getSPP(); i++) {
		//			float px = x + 0.5f;
		//			float py = y + 0.5f;
		//			Ray ray = scene->camera.generateRay(px, py);
		//			//Colour col = viewNormals(ray);
		//			//Colour col = albedo(ray);
		//			//Colour col = direct(ray, samplers);
		//			Colour throughput = Colour(1.0f, 1.0f, 1.0f);
		//			Colour col = pathTrace(ray, throughput, 0, samplers);
		//			film->splat(px, py, col);
		//		}
		//	}
		//}
		for (int i = 0; i < 512; i++) {
			film->incrementSPP();
			for (int y = 0; y < film->height; y++) {
				for (int x = 0; x < film->width; x++) {
					float px = x + samplers->next();
					float py = y + samplers->next();
					Ray ray = scene->camera.generateRay(px, py);
					//Colour col = viewNormals(ray);
					//Colour col = albedo(ray);
					//Colour col = direct(ray, samplers);
					Colour throughput = Colour(1.0f, 1.0f, 1.0f);
					Colour col = pathTrace(ray, throughput, 0, samplers);
					film->splat(px, py, col);
				}
			}
			canvas->clear();
			for (int y = 0; y < film->height; y++) {
				for (int x = 0; x < film->width; x++) {
					unsigned char r;
					unsigned char g;
					unsigned char b;
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
			canvas->present();
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