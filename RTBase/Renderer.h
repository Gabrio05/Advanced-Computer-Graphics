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
	void renderPixel(const int x, const int y, const int thread_number = 0) {
		float px = x + samplers[thread_number].next();
		float py = y + samplers[thread_number].next();
		Ray ray = scene->camera.generateRay(px, py);
		//Colour col = viewNormals(ray);
		//Colour col = albedo(ray);
		//Colour col = direct(ray, samplers);
		Colour throughput = Colour(1.0f, 1.0f, 1.0f);
		Colour col = pathTrace(ray, throughput, 0, &samplers[thread_number]);
		film->splat(px, py, col);
	}
	void renderTile(const int tile_size_horizontal, const int tile_size_vertical, std::atomic<int>& tile_index, const int thread_number) {
		int horizontal_tiles = film->width / tile_size_horizontal + (film->width % tile_size_horizontal != 0);
		int vertical_tiles = film->height / tile_size_vertical + (film->height % tile_size_vertical != 0);
		int current_index = tile_index++;
		int spp = getSPP();
		while (current_index < horizontal_tiles * vertical_tiles) {
			int tile_height = current_index / horizontal_tiles;
			int tile_width = current_index % horizontal_tiles;
			for (int y = tile_height * tile_size_vertical; y < (tile_height + 1) * tile_size_vertical && y < film->height; y++) {
				for (int x = tile_width * tile_size_horizontal; x < (tile_width + 1) * tile_size_horizontal && x < film->width; x++) {
					for (int i = 0; i < spp; i++) {
						renderPixel(x, y, thread_number);
					}
				}
			}
			current_index = tile_index++;
		}
	}
	void render()
	{
		film->SPP = 1;
		// Multi Threaded
		int thread_number = numProcs;
		int tile_size_horizontal = 20;  // Number of pixels per tile side
		int tile_size_vertical = 20;
		std::atomic<int> tile_index = 0;
		std::vector<std::thread> threads{};
		threads.reserve(thread_number);
		for (int t = 0; t < thread_number; t++) {
			threads.emplace_back(&RayTracer::renderTile, this, tile_size_horizontal, tile_size_vertical, std::ref(tile_index), t);
		}
		for (int t = 0; t < thread_number; t++) {
			threads[t].join();
		}

		// Single Threaded
		//for (int y = 0; y < film->height; y++) {
		//	for (int x = 0; x < film->width; x++) {
		//		for (int i = 0; i < getSPP(); i++) {
		//			renderPixel(x, y);
		//		}
		//	}
		//}
		// Continuous Rendering Single Threaded
		//for (int i = 0; i < 512; i++) {
		//	film->incrementSPP();
		//	for (int y = 0; y < film->height; y++) {
		//		for (int x = 0; x < film->width; x++) {
		//			renderPixel(x, y);
		//		}
		//	}
		//	canvas->clear();
		//	for (int y = 0; y < film->height; y++) {
		//		for (int x = 0; x < film->width; x++) {
		//			unsigned char r;
		//			unsigned char g;
		//			unsigned char b;
		//			film->tonemap(x, y, r, g, b);
		//			canvas->draw(x, y, r, g, b);
		//		}
		//	}
		//	canvas->present();
		//}
		
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