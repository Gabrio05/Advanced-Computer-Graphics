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
//#define DENOISE
#ifdef DENOISE
#include "OpenImageDenoise/oidn.hpp"

class Denoiser {
public:
	Film* film;
	oidn::DeviceRef device;
	oidn::BufferRef colorBuf;
	oidn::BufferRef albedoBuf;
	oidn::BufferRef normalBuf;
	oidn::FilterRef filter;
	Denoiser(Film* _film) {
		film = _film;
		// Following denoiser code copied from OpenImageDenoise examples
		// Create an Open Image Denoise device
		device = oidn::newDevice(); // CPU or GPU if available
		// oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
		device.commit();
		// Create buffers for input/output images accessible by both host (CPU) and device (CPU/GPU)
		colorBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		albedoBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		normalBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		// Create a filter for denoising a beauty (color) image using optional auxiliary images too
		// This can be an expensive operation, so try no(sic) to create a new filter for every image!
		filter = device.newFilter("RT"); // generic ray tracing filter
		filter.setImage("color", colorBuf, oidn::Format::Float3, film->width, film->height); // beauty
		//filter.setImage("albedo", albedoBuf, oidn::Format::Float3, film->width, film->height); // auxiliary
		//filter.setImage("normal", normalBuf, oidn::Format::Float3, film->width, film->height); // auxiliary
		filter.setImage("output", colorBuf, oidn::Format::Float3, film->width, film->height); // denoised beauty
		filter.set("hdr", true); // beauty image is HDR
		filter.commit();
	}
	void fillImageBuffer(std::string buffer_name = "color") {
		oidn::BufferRef& buffer_to_modify = buffer_name == "albedo" ? albedoBuf : buffer_name == "normal" ? normalBuf : colorBuf;
		buffer_to_modify.write(0, film->width * film->height * 3 * sizeof(float), film->film);
	}
	void fillColourImageBufferAndExecute() {
		// Fill the input image buffers
		//float* colorPtr = (float*)colorBuf.getData();
		fillImageBuffer();

		// Filter the beauty image
		filter.execute();
		// Check for errors
		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None) {
			std::cout << "Error: " << errorMessage << std::endl;
		}
		colorBuf.read(0, film->width * film->height * 3 * sizeof(float), film->film);
	}
};
#endif

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
#ifdef DENOISE
	Denoiser* openImageDenoiser;
#endif
	int numProcs;
	std::string current_render = "direct";
	static constexpr bool is_using_instant_radiosity = false;  // Render must be set to direct if true
	static constexpr int lights_to_sample_for_IR = 0;  // 0 for all
	static constexpr int lights_to_make = 5;
	int original_light_vector_size = 0;
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
#ifdef DENOISE
		openImageDenoiser = new Denoiser(film);
		current_render = "albedo";
		render();
		openImageDenoiser->fillImageBuffer("albedo");
		current_render = "normal";
		render();
		openImageDenoiser->fillImageBuffer("normal");
		current_render = "color";
#endif
	}
	void clear()
	{
		film->clear();
	}
	void connectToCamera(Vec3 point, Vec3 point_normal, Colour colour) {
		if (is_using_instant_radiosity) {
			VirtualPointLight* light = new VirtualPointLight();
			light->emission = colour;
			light->normal_vector = point_normal;
			light->position = point;
			scene->lights.emplace_back(light);
			return;
		}
		float x;
		float y;
		if (!scene->camera.projectOntoCamera(point, x, y) || !scene->visible(point, scene->camera.origin)) { return; }
		Vec3 dir_to_camera = (scene->camera.origin - point).normalize();
		float geo = std::max(Dot(dir_to_camera, point_normal), 0.0f) * std::max(Dot(-dir_to_camera, scene->camera.viewDirection), 0.0f) / (point - scene->camera.origin).lengthSq();
		float W = 1 / (scene->camera.Afilm * powf(Dot(scene->camera.viewDirection, dir_to_camera), 4));
		film->splat(x, y, colour * geo * W);
	}
	void lightPathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, ShadingData* old_shading = nullptr) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t >= FLT_MAX) { return; }
		if (!shadingData.bsdf->isPureSpecular()) {
			connectToCamera(shadingData.x, shadingData.sNormal, shadingData.bsdf->evaluate(shadingData, (scene->camera.origin - shadingData.x).normalize()) * pathThroughput);
		}
		float epsilon = 0.0001f;
		float c = 0.0f;
		Colour reflectedColour{};
		float pdf = 0;
		Vec3 next_vector = shadingData.bsdf->sample(shadingData, sampler, reflectedColour, pdf);
		Ray next = Ray(r.at(intersection.t) + next_vector * epsilon, next_vector);
		float the_cos = Dot(shadingData.gNormal, next_vector);
		pathThroughput = pathThroughput * reflectedColour * the_cos / pdf;
		float rrp = std::min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() > rrp) {
			return;
		}
		float qc = (1 - pathThroughput.Lum()) * c;
		lightPathTrace(next, pathThroughput, depth + 1, sampler, &shadingData);
	}
	void lightTrace(Sampler* sampler) {
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		Ray next;
		Colour pathThroughput = Colour(1, 1, 1);
		Colour colour;
		Vec3 light_point;
		float pdf_light;
		Vec3 direction;
		if (light->isArea()) {
			float pdf;
			light_point = light->sample(ShadingData{}, sampler, colour, pdf);
			connectToCamera(light_point, light->normal(ShadingData{}, Vec3{}), colour / (pmf * pdf));
			float dir_pdf;
			direction = light->sampleDirectionFromLight(sampler, dir_pdf);
			pdf_light = pmf * pdf * dir_pdf;
		} else {
			float pdf;
			direction = -light->sample(ShadingData{}, sampler, colour, pdf);
			connectToCamera(scene->camera.origin - direction * 100000000, light->normal(ShadingData{}, direction), colour / (pmf * pdf));
			Vec3 min_bounds = scene->bvh->bounds.min;
			Vec3 max_bounds = scene->bvh->bounds.max;
			std::vector<float> dot_values{};
			for (int i = 0; i < 3; i++) {
				Vec3 compare = Vec3(0, 0, 0);
				compare.coords[i] = 1;
				dot_values.emplace_back(Dot(direction, compare));
			}
			int highest_index = 0;
			float highest_dot_value = 0;
			for (int i = 0; i < 3; i++) {
				if (abs(dot_values.at(i)) > abs(highest_dot_value)) {
					highest_dot_value = dot_values.at(i);
					highest_index = i;
				}
			}
			light_point = min_bounds;
			float point_pdf = 1.0f;
			if (highest_dot_value < 0) { light_point = max_bounds; }
			for (int i = 0; i < 3; i++) {
				if (i != highest_index) {
					light_point.coords[i] = min_bounds.coords[i] + (max_bounds.coords[i] - min_bounds.coords[i]) * sampler->next();
					point_pdf /= max_bounds.coords[i] - min_bounds.coords[i];
				}
			}
			pdf_light = pdf * pmf * point_pdf;
			next = Ray(light_point, direction);
			pathThroughput = pathThroughput * colour / pdf_light;
			lightPathTrace(next, pathThroughput, 1, sampler);
			return;
		}
		next = Ray(light_point, direction);
		float the_cos = Dot(light->normal(ShadingData{}, direction), direction);
		pathThroughput = pathThroughput * colour * the_cos / pdf_light;
		lightPathTrace(next, pathThroughput, 1, sampler);
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler, int light_index = 0)
	{
		if (shadingData.bsdf->isPureSpecular()) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (is_using_instant_radiosity && lights_to_sample_for_IR == 0) {
			light = scene->lights[light_index];
			pmf = 1.0f;
		}
		if (light->isArea()) {
			float pdf;
			Colour colour;
			Vec3 light_point = light->sample(shadingData, sampler, colour, pdf);
			float pdf_light = pdf * pmf;
			Vec3 direction = (light_point - shadingData.x).normalize();
			float geo = std::max(Dot(direction, shadingData.sNormal), 0.0f) * std::max(Dot(-direction, light->normal(shadingData, direction)), 0.0f) / (light_point - shadingData.x).lengthSq();
			if (geo > 0 && scene->visible(shadingData.x, light_point)) {
				Colour colour2 = shadingData.bsdf->evaluate(shadingData, direction);
				float bsdf_pdf = shadingData.bsdf->PDF(shadingData, direction) * std::max(Dot(-direction, light->normal(shadingData, direction)), 0.0f) / (light_point - shadingData.x).lengthSq();
				//return colour * colour2 * geo / (pmf * pdf);
				return colour * colour2 * geo / (pdf_light + bsdf_pdf);
			}
		} else {
			float pdf;
			Colour colour;
			Vec3 light_direction = light->sample(shadingData, sampler, colour, pdf);
			float pdf_light = pdf * pmf;
			float geo = std::max(Dot(light_direction, shadingData.sNormal), 0.0f);
			if (geo > 0 && scene->visible(shadingData.x, shadingData.x + light_direction * 100000000)) {
				Colour colour2 = shadingData.bsdf->evaluate(shadingData, light_direction);
				float bsdf_pdf = shadingData.bsdf->PDF(shadingData, light_direction);
				//return colour * colour2 * geo / (pmf * pdf);
				return colour * colour2 * geo / (pdf_light + bsdf_pdf);
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, ShadingData* old_shading = nullptr)
	{
		// Direct
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		Colour directLight = Colour(0, 0, 0);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				if (depth == 0 || old_shading->bsdf->isPureSpecular()) {
					return shadingData.bsdf->emit(shadingData, shadingData.wo) * pathThroughput;
				}
				else {
					//return Colour(0, 0, 0);
					auto it = std::lower_bound(scene->light_indices.begin(), scene->light_indices.end(), intersection.ID);
					int light_index = it - scene->light_indices.begin();
					float light_pdf = scene->pmfSampleLight(scene->lights[light_index]) * scene->lights[light_index]->PDF(shadingData, r.dir);
					float bsdf_pdf = old_shading->bsdf->PDF(*old_shading, r.dir) * std::max(Dot(-r.dir, shadingData.sNormal), 0.0f) / (shadingData.x - old_shading->x).lengthSq();
					return shadingData.bsdf->emit(shadingData, shadingData.wo) * pathThroughput / (light_pdf + bsdf_pdf);
				}
			}
			//constexpr int i = is_using_instant_radiosity ? lights_to_sample_for_IR : 1;
			//if (i == 0) {
			//	for (int j = 0; j < scene->lights.size(); j++) {
			//		directLight = directLight + computeDirect(shadingData, sampler, j);
			//	}
			//	directLight = directLight / scene->lights.size();
			//} else {
			//	int lights_to_do = i;
			//	while (lights_to_do > 0) {
			//		directLight = directLight + computeDirect(shadingData, sampler);
			//		lights_to_do--;
			//	}
			//	directLight = directLight / i;
			//}
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
		Ray next = Ray(r.at(intersection.t) + next_vector * epsilon, next_vector);
		float the_cos = Dot(shadingData.gNormal, next_vector);
		pathThroughput = pathThroughput * reflectedColour * the_cos / pdf;
		float rrp = std::min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() > rrp) {
			return directLight + Colour(c, c, c) * pathThroughput;
		}
		float qc = (1 - pathThroughput.Lum()) * c;
		return directLight + (pathTrace(next, pathThroughput, depth + 1, sampler, &shadingData) - Colour(qc, qc, qc)) / (rrp);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		Colour directLight = Colour(0, 0, 0);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			constexpr int i = is_using_instant_radiosity ? lights_to_sample_for_IR : 1;
			if (i == 0) {
				for (int j = original_light_vector_size; j < scene->lights.size(); j++) {
					directLight = directLight + computeDirect(shadingData, sampler, j);
				}
				directLight = directLight / (scene->lights.size() - original_light_vector_size) / lights_to_make;
			} else {
				int lights_to_do = i;
				while (lights_to_do > 0) {
					directLight = directLight + computeDirect(shadingData, sampler);
					lights_to_do--;
				}
				directLight = directLight / i;
			}
			return directLight;
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
		Colour col;
		if (current_render == "normal") {
			col = viewNormals(ray);
		} else if (current_render == "albedo") {
			col = albedo(ray);
		} else if (current_render == "direct") {
			col = direct(ray, samplers);
		} else {
			Colour throughput = Colour(1.0f, 1.0f, 1.0f);
			col = pathTrace(ray, throughput, 0, &samplers[thread_number]);
		}
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
		// No multi-threaded light tracing without locks! Don't you dare!
		//if (is_using_instant_radiosity) {
		//	original_light_vector_size = scene->lights.size();
		//	for (int i = 0; i < lights_to_make; i++) {
		//		lightTrace(&samplers[0]);
		//	}
		//}
		//int thread_number = numProcs;
		//int tile_size_horizontal = 20;  // Number of pixels per tile side
		//int tile_size_vertical = 20;
		//std::atomic<int> tile_index = 0;
		//std::vector<std::thread> threads{};
		//threads.reserve(thread_number);
		//for (int t = 0; t < thread_number; t++) {
		//	threads.emplace_back(&RayTracer::renderTile, this, tile_size_horizontal, tile_size_vertical, std::ref(tile_index), t);
		//}
		//for (int t = 0; t < thread_number; t++) {
		//	threads[t].join();
		//}

		// Single Threaded
		for (int y = 0; y < film->height; y++) {
			for (int x = 0; x < film->width; x++) {
				for (int i = 0; i < getSPP(); i++) {
					//renderPixel(x, y);
					lightTrace(&samplers[0]);
				}
			}
		}
		// Continuous Rendering Single Threaded
		//for (int i = 0; i < 1; i++) {
		//	film->incrementSPP();
		//	for (int y = 0; y < film->height; y++) {
		//		for (int x = 0; x < film->width; x++) {
		//			//renderPixel(x, y);
		//			lightTrace(&samplers[0]);
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
#ifdef DENOISE
		openImageDenoiser->fillColourImageBufferAndExecute();
#endif
		
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
		if (is_using_instant_radiosity) {
			while (scene->lights.size() > original_light_vector_size) {
				delete scene->lights.back();
				scene->lights.pop_back();
			}
		}
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