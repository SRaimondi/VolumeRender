/*
 * ScalarField.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef SCALARFIELD_H_
#define SCALARFIELD_H_

#include "Geometry.h"
#include "Interpolation.h"

// Interpolation mode used
enum class INTERP_MODE {
	LINEAR,
	CUBIC,
};

// Out of bound mode
enum class BOUND_MODE {
	ZERO,
};

// This struct hold the description of a scalar field
class ScalarFieldDescription {
public:
	// Dimensions
	int dims[3];
	// Bounds
	BBOX bounds;
	// Voxel dimension
	Vec3 voxel_dim;
	// Interpolation mode
	INTERP_MODE inter_mode = INTERP_MODE::LINEAR;
	// Out of bounds mode
	BOUND_MODE bound_mode = BOUND_MODE::ZERO;
	// Maximum and minimum value of the field
	float field_min, field_max;

	// Constructor
	CUDA_HOST_DEVICE ScalarFieldDescription(int nx, int ny, int nz, const BBOX &bounds,
											const INTERP_MODE &i_m) noexcept;

	// Compute value using tri-linear interpolation
	CUDA_HOST_DEVICE float LinearInterpolate(const float* data, const Vec3 &p) const noexcept;

	// Compute value using tri-cubic interpolation
	CUDA_HOST_DEVICE float CubicInterpolate(const float* data, const Vec3 &p) const noexcept;

	// Get field value at given coordinates
	CUDA_HOST_DEVICE inline float operator()(const float* data, int i, int j, int k) const noexcept {
		// Check if we are out of boundaries
		if (i < 0 || i >= dims[0] ||
			j < 0 || j >= dims[1] ||
			k < 0 || k >= dims[2]) {
			// Check the mode we want to use
			switch (bound_mode) {
			case BOUND_MODE::ZERO: return 0.f;
			}
		}
		return data[Offset(i, j, k)];
	}

	// Compute field value at given 3D point
	CUDA_HOST_DEVICE inline float At(const float* data, const Vec3 &p) const noexcept {
		// Check if we are inside the boundaries
		if (!bounds.Inside(p)) {
			switch (bound_mode) {
			case BOUND_MODE::ZERO: return 0.f;
			}
		}
		// Interpolate value using selected mode
		switch (inter_mode) {
		case INTERP_MODE::LINEAR: return LinearInterpolate(data, p);
		case INTERP_MODE::CUBIC: return CubicInterpolate(data, p);
		default: return 0.f;	// Function will return zero if the interpolation mode is not recognized
		}
	}

	// Compute linear offset
	CUDA_HOST_DEVICE inline int Offset(int i, int j, int k) const noexcept {
		return (k * (dims[1] - 1) + j) * (dims[0] - 1) + i;
	}

	// Compute the index of the voxel for a given 3D point
	CUDA_HOST_DEVICE inline int Pos2Voxel(const Vec3 &p, int axis) const noexcept {
		const int v_i = int(std::floor((p[axis] - bounds[0][axis]) / voxel_dim[axis]));
		return Clamp(v_i, 0, dims[axis] - 2);
	}

	// Set data value at given position
	CUDA_HOST inline void Set(float* data, float v, int i, int j, int k) const noexcept {
		if (i >= 0 && i < dims[0] &&
			j >= 0 && j < dims[1] &&
			k >= 0 && k < dims[2]) {
			data[Offset(i, j, k)] = v;
		}
	}

	// Update maximum and minimum of the field
	CUDA_HOST void UpdateMinMax(const float* data) noexcept;

	// Intersect ray with the volume BBOX
	CUDA_HOST_DEVICE inline bool Intersect(const Ray &ray, float &t_min, float &t_max) const noexcept {
		return bounds.Intersect(ray, t_min, t_max);
	}

	// Normalize data
	CUDA_HOST void Normalize(float* data) noexcept;
};


#endif /* SCALARFIELD_H_ */
