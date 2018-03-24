/*
 * Geometry.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include "Common.h"
#include <cmath>

class Vec3 {
public:
	// Vector components
	union {
		struct {
			float x, y, z;
		};
		float e[3];
	};

	// Constructors
	CUDA_HOST_DEVICE inline constexpr Vec3() noexcept
		: x(0), y(0), z(0) {
	}

	CUDA_HOST_DEVICE inline explicit Vec3(float v) noexcept
		: x(v), y(v), z(v) {
	}

	CUDA_HOST_DEVICE Vec3(float x, float y, float z) noexcept
		: x(x), y(y), z(z) {
	}

	// Math operators
	CUDA_HOST_DEVICE inline Vec3 &operator+=(const Vec3 &v) noexcept {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	CUDA_HOST_DEVICE  inline Vec3 &operator-=(const Vec3 &v) noexcept {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	CUDA_HOST_DEVICE inline Vec3 &operator*=(const Vec3 &v) noexcept {
		x *= v.x;
		y *= v.y;
		z *= v.z;
		return *this;
	}

	// Index operator
	CUDA_HOST_DEVICE inline float operator[](int i) const noexcept {
		return e[i];
	}

	CUDA_HOST_DEVICE inline float &operator[](int i) noexcept {
		return e[i];
	}
};

// Operators
CUDA_HOST_DEVICE inline Vec3 operator+(const Vec3 &v1, const Vec3 &v2) noexcept {
	return{ v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

CUDA_HOST_DEVICE inline Vec3 operator-(const Vec3 &v1, const Vec3 &v2) noexcept {
	return{ v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

CUDA_HOST_DEVICE inline Vec3 operator-(const Vec3 &v) noexcept {
	return{ -v.x, -v.y, -v.z };
}

CUDA_HOST_DEVICE inline Vec3 operator*(const Vec3 &v1, const Vec3 &v2) noexcept {
	return{ v1.x * v2.x, v1.y * v2.y, v1.z * v2.z };
}

CUDA_HOST_DEVICE inline Vec3 operator/(const Vec3 &v1, const Vec3 &v2) noexcept {
	return{ v1.x / v2.x, v1.y / v2.y, v1.z / v2.z };
}

CUDA_HOST_DEVICE inline Vec3 operator*(float t, const Vec3 &v) noexcept {
	return{ t * v.x, t * v.y, t * v.z };
}

CUDA_HOST_DEVICE inline Vec3 operator*(const Vec3 &v, float t) noexcept {
	return{ t * v.x, t * v.y, t * v.z };
}

CUDA_HOST_DEVICE inline Vec3 operator/(const Vec3 &v, float t) noexcept {
	const float inv = 1.f / t;
	return inv * v;
}

// Dot product
CUDA_HOST_DEVICE inline float Dot(const Vec3 &v1, const Vec3 &v2) noexcept {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// Norm squared
CUDA_HOST_DEVICE inline float Norm2(const Vec3 &v) noexcept {
	return Dot(v, v);
}

// Norm
CUDA_HOST_DEVICE inline float Norm(const Vec3 &v) noexcept {
	return std::sqrt(Norm2(v));
}

// Normalize
CUDA_HOST_DEVICE inline Vec3 Normalize(const Vec3 &v) noexcept {
	return v / Norm(v);
}

// Cross product
CUDA_HOST_DEVICE inline Vec3 Cross(const Vec3 &v1, const Vec3 &v2) noexcept {
	return{ v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x };
}

// Component-wise min and max
CUDA_HOST_DEVICE inline Vec3 Min(const Vec3 &v1, const Vec3 &v2) noexcept {
	return{ FMin(v1.x, v2.x), FMin(v1.y, v2.y), FMin(v1.z, v2.z) };
}

CUDA_HOST_DEVICE inline Vec3 Max(const Vec3 &v1, const Vec3 &v2) noexcept {
	return{ FMax(v1.x, v2.x), FMax(v1.y, v2.y), FMax(v1.z, v2.z) };
}

// Vector exponential
CUDA_HOST_DEVICE inline Vec3 Exp(const Vec3 &v) noexcept {
	return{ std::exp(v.x), std::exp(v.y), std::exp(v.z) };
}

// Permute vector
CUDA_HOST_DEVICE inline Vec3 Permute(const Vec3 &v, int x, int y, int z) noexcept {
	return{ v[x], v[y], v[z] };
}

// Ray class
class Ray {
public:
	// Origin
	Vec3 o;
	// Direction
	Vec3 d;

	// Constructors
	CUDA_HOST_DEVICE inline constexpr Ray() noexcept
		: o(), d() {
	}

	CUDA_HOST_DEVICE inline Ray(const Vec3 &o, const Vec3 &d) noexcept
		: o(o), d(d) {
	}

	// Evaluate point on ray given parameter
	CUDA_HOST_DEVICE inline Vec3 operator()(float t) const noexcept {
		return o + t * d;
	}
};

// BBOX class
class BBOX {
private:
	// Boundaries
	Vec3 bounds[2];

public:
	// Constructors
	CUDA_HOST_DEVICE inline BBOX() noexcept {
		bounds[0] = Vec3(NEG_INF);
		bounds[1] = Vec3(INF);
	}

	CUDA_HOST_DEVICE inline explicit BBOX(const Vec3 &p) noexcept {
		bounds[0] = p;
		bounds[1] = p;
	}

	CUDA_HOST_DEVICE inline BBOX(const Vec3 &a, const Vec3 &b) noexcept {
		bounds[0] = Min(a, b);
		bounds[1] = Max(a, b);
	}

	CUDA_HOST_DEVICE inline const Vec3 &operator[](int i) const noexcept {
		return bounds[i];
	}

	// Diagonal
	CUDA_HOST_DEVICE inline Vec3 Diagonal() const noexcept {
		return bounds[1] - bounds[0];
	}

	// Check if a point is inside
	CUDA_HOST_DEVICE inline bool Inside(const Vec3 &p) const noexcept {
		return (p.x >= bounds[0].x && p.x <= bounds[1].x &&
				p.y >= bounds[0].y && p.y <= bounds[1].y &&
				p.z >= bounds[0].z && p.z <= bounds[1].z);
	}

	CUDA_HOST_DEVICE inline bool Intersect(const Ray &ray,
										   float& t_min,
										   float& t_max) const noexcept {
		t_min = 0;
		t_max = INF;
		for (int i = 0; i < 3; ++i) {
			// Update interval for current slab
			const float inv_ray_dir = 1.f / ray.d[i];
			float t_near = (bounds[0][i] - ray.o[i]) * inv_ray_dir;
			float t_far = (bounds[1][i] - ray.o[i]) * inv_ray_dir;

			// Check if we need swap
			if (t_near > t_far) {
				Swap(t_near, t_far);
			}

			// Update interval
			t_min = t_near > t_min ? t_near : t_min;
			t_max = t_far < t_max ? t_far : t_max;

			// Check if we are out of BBOX
			if (t_min > t_max) {
				return false;
			}
		}
		// We have a hit
		return true;
	}
};

#endif /* GEOMETRY_H_ */
