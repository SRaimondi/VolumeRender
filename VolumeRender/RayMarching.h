/*
 * RayMarching.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef RAYMARCHING_H_
#define RAYMARCHING_H_

#include "Camera.h"
#include "ScalarField.h"
#include "Film.h"
#include "TransferFunction.h"

 // Initial RayMarching function
template <typename TF_FUNC>
CUDA_GLOBAL void RayMarchVolume(
	const ScalarFieldDescription* field_description,	// Description of the scalar field to render
	const float* field_data,							// Actual field data
	const Camera* camera,								// Camera to use
	const FilmDescription* film_description,			// Description of the film used
	Spectrum* film_raster,								// Film raster
	float marching_delta,								// Step to use in the marching process
	const TF_FUNC* transfer_function,                   // Transfer function
	const TF1DControlPoint* tf_control_points           //Transfer function control points
) {
	// Get thread ids
	const volatile unsigned idx = threadIdx.x + blockIdx.x * blockDim.x;
	const volatile unsigned idy = threadIdx.y + blockIdx.y * blockDim.y;

	if (idx < film_description->Width() && idy < film_description->Height()) {
		// Generate ray for this pixel
		const Ray ray = camera->GenerateRay(idx, idy, 0.5f, 0.5f);
		// Incoming radiance value for the ray
		Spectrum L(1.f);

		// Minimum and maximum intersection parameters of the ray with BBOX
		float t_min, t_max;
		if (field_description->Intersect(ray, t_min, t_max)) {
			// We hit the volume, start ray marching
			while (t_min <= t_max) {
				// Get volume density at current point
				float density = field_description->At(field_data, ray(t_min));
				// Evaluate transfer function
				const TFOutput tf_value = field_description->is_normalized ?
					transfer_function->At(tf_control_points, density) :
					transfer_function->At(tf_control_points, field_description->NormalizeVal(density));
				// Compute integral
				L *= Exp(-tf_value.density * marching_delta * tf_value.attenuation);
				// Next step
				t_min += marching_delta;
			}
		}
		// Set film value
		film_description->Set(film_raster, L, idx, idy);
	}
}

// Ray march scalar field using empty space map
template <typename TF_FUNC>
CUDA_GLOBAL void RayMarchVolumeEmptySpaceMap(
	const ScalarFieldDescription* field_description,	// Description of the scalar field to render
	const float* field_data,							// Actual field data
	const bool* empty_space_map,                        // Empty space map
	const Camera* camera,								// Camera to use
	const FilmDescription* film_description,			// Description of the film used
	Spectrum* film_raster,								// Film raster
	float marching_delta,								// Step to use in the marching process
	const TF_FUNC* transfer_function,                   // Transfer function
	const TF1DControlPoint* tf_control_points           //Transfer function control points
) {
	// Get thread ids
	const volatile unsigned idx = threadIdx.x + blockIdx.x * blockDim.x;
	const volatile unsigned idy = threadIdx.y + blockIdx.y * blockDim.y;

	if (idx < film_description->Width() && idy < film_description->Height()) {
		// Generate ray for this pixel
		const Ray ray = camera->GenerateRay(idx, idy, 0.5f, 0.5f);
		// Incoming radiance value for the ray
		Spectrum L(1.f);

		// Minimum and maximum intersection parameters of the ray with BBOX
		float t_min, t_max;
		if (field_description->Intersect(ray, t_min, t_max)) {
			// We hit the volume, start ray marching
			while (t_min <= t_max) {
				// Current point
				const Vec3 p = ray(t_min);
				// Compute current voxel indices
				int v_i[3];
				for (int axis = 0; axis < 3; ++axis) {
					v_i[axis] = field_description->Pos2Voxel(p, axis);
				}
				// Check if the voxel is empty from out map
				if (empty_space_map[field_description->Offset(v_i[0], v_i[1], v_i[2])]) {
					// Compute voxel minimum and maximum point
					const Vec3 v_min(field_description->bounds[0].x + v_i[0] * field_description->voxel_dim.x,
									 field_description->bounds[0].y + v_i[1] * field_description->voxel_dim.y,
									 field_description->bounds[0].z + v_i[2] * field_description->voxel_dim.z);
					const Vec3 v_max = v_min + field_description->voxel_dim;
					// Compute in and out value for the voxel
					float voxel_min, voxel_max;
					// We might get some precision problem, so we need to clamp our travel value
					(void)BBOX(v_min, v_max).Intersect(ray, voxel_min, voxel_max);
					// Travel as much as we can
					t_min += FMax(voxel_max - voxel_min, marching_delta);
				} else {
					// Get volume density at current point
					const float density = field_description->At(field_data, p);
					// Evaluate transfer function
					const TFOutput tf_value = field_description->is_normalized ?
						transfer_function->At(tf_control_points, density) :
						transfer_function->At(tf_control_points, field_description->NormalizeVal(density));
					// Compute integral
					L *= Exp(-tf_value.density * marching_delta * tf_value.attenuation);
					// Next step
					t_min += marching_delta;
				}
			}
		}
		// Set film value
		film_description->Set(film_raster, L, idx, idy);
	}
}

#endif /* RAYMARCHING_H_ */
