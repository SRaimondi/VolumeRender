#include "RayMarching.h"

CUDA_GLOBAL void RayMarchVolume(
	const ScalarFieldDescription* field_description,
	const float* field_data,
	const Camera* camera,
	const FilmDescription* film_description,
	Spectrum* film_raster,
	float marching_delta) {
	// Get thread ids
	const volatile unsigned idx = threadIdx.x + blockIdx.x * blockDim.x;
	const volatile unsigned idy = threadIdx.y + blockIdx.y * blockDim.y;

	if (idx < film_description->Width() && idy < film_description->Height()) {
		// Generate ray for this pixel
		const Ray ray = camera->GenerateRay(idx, idy, 0.5f, 0.5f);
		// Incoming radiance value for the ray
		Spectrum L(1.f);

		// Hardcoded "transfer function"
		const Spectrum sigma_a[3] = { Spectrum(0.1, 0.7, 0.1),
			Spectrum(0.2, 0.3, 0.7),
			Spectrum(0.1, 0.1, 0.5) };

		// Minimum and maximum intersection parameters of the ray with BBOX
		float t_min, t_max;
		if (field_description->Intersect(ray, t_min, t_max)) {
			// We hit the volume, start ray marching
			while (t_min <= t_max) {
				// Get volume density at current point
				const float density = field_description->At(field_data, ray(t_min));
				// Attenuate color based on density and transfer function TODO
				if (Approx(density, 0.17, 0.6)) {
					L *= Exp(-3 * density * marching_delta * (Spectrum(1) - sigma_a[0]));
				} else if (Approx(density, 0.06, 0.2)) {
					L *= Exp(-3 * density * marching_delta * (Spectrum(1) - sigma_a[1]));
				} else if (Approx(density, 0.1, 0.2)) {
					L *= Exp(-3 * density * marching_delta * (Spectrum(1) - sigma_a[2]));
				}
				// Next step
				t_min += marching_delta;
			}
		}
		// Set film value
		film_description->Set(film_raster, L, idx, idy);
	}
}



