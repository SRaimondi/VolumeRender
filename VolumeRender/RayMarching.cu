#include "RayMarching.h"

CUDA_GLOBAL void RayMarchVolume(
	const ScalarFieldDescription* field_description,
	const float* field_data,
	const Camera* camera,
	const FilmDescription* film_description,
	Spectrum* film_raster,
	float marching_delta,
	const TF1DCubic* transfer_function,
	const TF1DControlPoint* tf_control_points) {
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



