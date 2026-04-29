#pragma once

#include "Core.h"
#include "Sampling.h"
#include <span>
#include <ranges>

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(const Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	bool rayIntersect(Ray& r, float& t)
	{
		t = -(n.dot(r.o) + d) / n.dot(r.dir);
		return t > 0.0f;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1;  // Edge 1
	Vec3 e2;  // Edge 2
	Vec3 n;  // Geometric Normal
	float area;  // Triangle area
	float d;  // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t <= 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
	}
	bool mollerRayIntersect(const Ray& r, float& t, float& u, float& v) const {
		Vec3 p = Cross(r.dir, e2);
		float det = Dot(e1, p);
		if (fabs(det) < 0.000001f) { return false; }
		Vec3 tre = r.o - vertices[0].p;
		u = Dot(tre, p) / det;
		if (u < 0 || u > 1) { return false; }
		Vec3 q = Cross(tre, e1);
		v = Dot(r.dir, q) / det;
		if (v < 0 || u + v > 1) { return false; }
		t = Dot(e2, q) / det;
		if (t <= 0) { return false; }
		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		pdf = 1 / area;
		float r1 = sampler->next();
		float r2 = sampler->next();
		float alpha = 1 - sqrtf(r1);
		float beta = r2 * sqrtf(r1);
		return vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * (1 - beta - alpha);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	void extend(const Triangle t) {
		extend(t.vertices[0].p);
		extend(t.vertices[1].p);
		extend(t.vertices[2].p);
	}
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 t_min = (min - r.o) * r.invDir;
		Vec3 t_max = (max - r.o) * r.invDir;
		Vec3 t_entry = Min(t_min, t_max);
		Vec3 t_exit = Max(t_min, t_max);
		t = std::max(t_entry.x, std::max(t_entry.y, t_entry.z));
		float t_ex = std::min(t_exit.x, std::min(t_exit.y, t_exit.z));
		if (t < t_ex && t_ex > 0) { return true; }
		return false;
	}
	bool rayAABB(const Ray& r)
	{
		float t = 0.0f;
		return rayAABB(r, t);
	}
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
	bool inBounds(const Vec3& vector) const {
		return vector.x < max.x && vector.y < max.y && vector.z < max.z
			&& vector.x > min.x && vector.y > min.y && vector.z > min.z;
	}
	//bool inBounds(const Triangle& t) {  // Even partially
	//	AABB bounds{};
	//	bounds.extend(t);
	//	if () {
	//		return false;
	//	}
	//	return true;
	//}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	bool rayIntersect(Ray& r, float& t)
	{
		float c = (r.o - centre).lengthSq() - radius * radius;
		float b = 2 * (r.dir * (r.o - centre)).length();
		float discriminant = b * b - 4 * c;
		if (discriminant > 0) {
			float t1 = (-b + sqrtf(discriminant)) / 2;
			float t2 = (-b - sqrtf(discriminant)) / 2;
			t = std::min(t1, t2);
			if (t > 0) { return true; }
			t = std::max(t1, t2);
			if (t > 0) { return true; }
		}
		else if (discriminant == 0) {
			t = -b / 2;
			if (t > 0) { return true; }
		}
		return false;		
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	std::unique_ptr<BVHNode> l;
	std::unique_ptr<BVHNode> r;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	int start_index = -1;
	int triangles_number = 0;
	BVHNode()
	{
		l = nullptr;
		r = nullptr;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::span<Triangle>& inputTriangles, int original_offset = 0)
	{
		for (const Triangle& triangle : inputTriangles) {
			bounds.extend(triangle);
		}

		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			start_index = original_offset; 
			triangles_number = static_cast<int>(inputTriangles.size());
			return;
		}

		Vec3 side_lengths = bounds.max - bounds.min;
		int axis = 0;
		if (side_lengths.x > side_lengths.y && side_lengths.x > side_lengths.z) {
			float new_x = bounds.max.x - side_lengths.x / 2;
			axis = 0;
		} else if (side_lengths.y > side_lengths.z) {
			float new_y = bounds.max.y - side_lengths.y / 2;
			axis = 1;
		} else {
			float new_z = bounds.max.z - side_lengths.z / 2;
			axis = 2;
		}
		std::sort(inputTriangles.begin(), inputTriangles.end(), [axis](const Triangle& t, const Triangle& t2) { return t.centre().coords[axis] < t2.centre().coords[axis]; });
		
		// Median
		int half = inputTriangles.size() / 2;
		// SAH
		//float best_cost = INFINITY;
		//int best_split = half;
		//const float left_min = bounds.min.coords[axis];
		//float left_max = bounds.min.coords[axis];
		//float right_min = bounds.min.coords[axis];
		//const float right_max = bounds.max.coords[axis];
		//int current_left_triangle 
		//for (int i = 0; i < inputTriangles.size(); i++) {
		//	
		//}

		std::span<Triangle> first_half = inputTriangles.subspan(0, half);
		std::span<Triangle> second_half = inputTriangles.subspan(half, inputTriangles.size() - half);
		l = std::make_unique<BVHNode>();
		r = std::make_unique<BVHNode>();
		l->build(first_half, original_offset);
		r->build(second_half, original_offset + half);
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection, const bool should_terminate_early = false, const float maxT = 0.0f)
	{
		if (bounds.rayAABB(ray)) {
			if (triangles_number > 0) {
				for (int i = start_index; i < start_index + triangles_number; i++) {
					float t;
					float u;
					float v;
					if (triangles[i].rayIntersect(ray, t, u, v)) {
						if (t < intersection.t) {
							intersection.t = t;
							intersection.ID = i;
							intersection.alpha = u;
							intersection.beta = v;
							intersection.gamma = 1.0f - (u + v);
							if (should_terminate_early && t < maxT) {
								return;
							}
						}
					}
				}
			} else {
				if (l != nullptr) {
					l->traverse(ray, triangles, intersection);
				}
				if (r != nullptr) {
					r->traverse(ray, triangles, intersection);
				}
			}
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles, const bool should_terminate_early = false, const float maxT = 0.0f)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection, should_terminate_early, maxT);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		return traverse(ray, triangles, true, maxT).t > maxT;
	}
};
