#include "ScalarField.h"
#include <iostream>

CUDA_HOST_DEVICE ScalarFieldDescription::ScalarFieldDescription(int nx, int ny, int nz, const BBOX &bounds,
																const INTERP_MODE &i_m) noexcept
	: bounds(bounds), inter_mode(i_m), field_min(INF), field_max(NEG_INF) {
	// Set dimensions
	dims[0] = nx;
	dims[1] = ny;
	dims[2] = nz;
	// Compute voxel size
	voxel_dim = bounds.Diagonal() / Vec3(nx - 1, ny - 1, nz - 1);
}

CUDA_HOST_DEVICE float ScalarFieldDescription::LinearInterpolate(const float* data, const Vec3 &p) const noexcept {
	const ScalarFieldDescription &field = *this;
	// Compute voxel coordinates
	int v_i[3] = { Pos2Voxel(p, 0),
		Pos2Voxel(p, 1),
		Pos2Voxel(p, 2) };

	// Compute minimum point of the voxel
	Vec3 v_min(bounds[0].x + v_i[0] * voxel_dim.x,
					 bounds[0].y + v_i[1] * voxel_dim.y,
					 bounds[0].z + v_i[2] * voxel_dim.z);

	// Compute offset inside voxel in range [0,1]x[0,1]x[0,1]
	Vec3 t = (p - v_min) / voxel_dim;

	// Compute final value using trilinear approximation
	float d0 = Lerp(t.x,
						  field(data, v_i[0], v_i[1], v_i[2]),
						  field(data, v_i[0] + 1, v_i[1], v_i[2]));
	float d1 = Lerp(t.x,
						  field(data, v_i[0], v_i[1] + 1, v_i[2]),
						  field(data, v_i[0] + 1, v_i[1] + 1, v_i[2]));
	float d2 = Lerp(t.x,
						  field(data, v_i[0], v_i[1] + 1, v_i[2] + 1),
						  field(data, v_i[0] + 1, v_i[1] + 1, v_i[2] + 1));
	float d3 = Lerp(t.x,
						  field(data, v_i[0], v_i[1], v_i[2] + 1),
						  field(data, v_i[0] + 1, v_i[1], v_i[2] + 1));

	float d01 = Lerp(t.y, d0, d1);
	float d23 = Lerp(t.y, d2, d3);

	return Lerp(t.z, d01, d23);
}

CUDA_HOST_DEVICE float ScalarFieldDescription::CubicInterpolate(const float* data, const Vec3 &p) const noexcept {
	const ScalarFieldDescription &field = *this;
	// Compute voxel coordinates
	int v_i[3] = { Pos2Voxel(p, 0),
		Pos2Voxel(p, 1),
		Pos2Voxel(p, 2) };

	// Compute minimum point of the voxel
	Vec3 v_min(bounds[0].x + v_i[0] * voxel_dim.x,
					 bounds[0].y + v_i[1] * voxel_dim.y,
					 bounds[0].z + v_i[2] * voxel_dim.z);

	// Compute offset inside voxel in range [0,1]x[0,1]x[0,1]
	Vec3 t = (p - v_min) / voxel_dim;

	// Compute interpolation values for z
	float z_interp_values[4][4];
	for (int j = -1; j < 3; ++j) {
		for (int i = -1; i < 3; ++i) {
			z_interp_values[j + 1][i + 1] = CINT_CR(t.z,
													field(data, v_i[0] + i, v_i[1] + j, v_i[2] - 1),
													field(data, v_i[0] + i, v_i[1] + j, v_i[2]),
													field(data, v_i[0] + i, v_i[1] + j, v_i[2] + 1),
													field(data, v_i[0] + i, v_i[1] + j, v_i[2] + 2));
		}
	}
	// Interpolate the previous values along y
	float y_interp_values[4];
	for (int i = 0; i < 4; ++i) {
		y_interp_values[i] = CINT_CR(t.y,
									 z_interp_values[0][i],
									 z_interp_values[1][i],
									 z_interp_values[2][i],
									 z_interp_values[3][i]);
	}

	// Return final interpolation
	return CINT_CR(t.x,
				   y_interp_values[0],
				   y_interp_values[1],
				   y_interp_values[2],
				   y_interp_values[3]);
}

CUDA_HOST void ScalarFieldDescription::UpdateMinMax(const float * data) noexcept {
	for (int i = 0; i < dims[0] * dims[1] * dims[2]; ++i) {
		if (data[i] < field_min) { field_min = data[i]; }
		if (data[i] > field_max) { field_max = data[i]; }
	}
}

CUDA_HOST void ScalarFieldDescription::Normalize(float * data) noexcept {
	// Update max and minimum
	UpdateMinMax(data);
	// Loop over data and normalize it
	for (int i = 0, float delta = field_max - field_min; i < dims[0] * dims[1] * dims[2]; ++i) {
		data[i] = (data[i] - field_min) / delta;
	}
}
