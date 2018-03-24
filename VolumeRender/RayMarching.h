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
				const float density = field_description->At(field_data, ray(t_min));
				// Evaluate transfer function
				const TFOutput tf_value = transfer_function->At(tf_control_points, density);
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

#endif /* RAYMARCHING_H_ */
