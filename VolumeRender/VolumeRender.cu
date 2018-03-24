#include "DDSLoader.h"
#include "RayMarching.h"
#include "TransferFunction.h"

#include <iostream>

int main(void) {
	// Create film
	constexpr int WIDTH = 512;
	constexpr int HEIGHT = 512;
	FilmDescription* film;
	CUDA_SAFE_CALL(cudaMallocManaged(&film, sizeof(FilmDescription)));
	*film = FilmDescription(WIDTH, HEIGHT);
	// Allocate space for film raster
	Spectrum* film_raster;
	CUDA_SAFE_CALL(cudaMallocManaged(&film_raster, WIDTH * HEIGHT * sizeof(Spectrum)));

	// Create camera
	Camera* camera;
	CUDA_SAFE_CALL(cudaMallocManaged(&camera, sizeof(Camera)));
	*camera = Camera(Vec3(1, 1, 2), Vec3(), Vec3(0, 1, 0), 60.f, 1.5f, WIDTH, HEIGHT);

	// Load DDS file
	unsigned char *volume = nullptr;
	unsigned int width, height, depth, components;
	Vec3 scale;
	if ((volume = readPVMvolume("../Bucky.pvm",
								&width, &height, &depth,
								&components,
								&scale.x, &scale.y, &scale.z)) == nullptr) {
		exit(EXIT_FAILURE);
	}

	// Scaling BBOX factor
	constexpr float scaling = 1.f;
	// Allocate scalar field description
	ScalarFieldDescription* scalar_field;
	CUDA_SAFE_CALL(cudaMallocManaged(&scalar_field, sizeof(ScalarFieldDescription)));
	*scalar_field = ScalarFieldDescription(width, height, depth,
										   BBOX(-0.5f * scaling * scale, 0.5f * scaling * scale),
										   INTERP_MODE::CUBIC);
	// Allocate space for field data
	float* scalar_field_data;
	CUDA_SAFE_CALL(cudaMallocManaged(&scalar_field_data, width * height * depth * sizeof(float)));

	// Fill volume data
	float sum = 0.f;
	// Fill scalar field
	for (unsigned int k = 0; k < depth; ++k) {
		for (unsigned int j = 0; j < height; ++j) {
			for (unsigned int i = 0; i < width; ++i) {
				scalar_field->Set(scalar_field_data,
								  float(volume[i + j * width + k * width * height]),
								  i, j, k);
				sum += (*scalar_field)(scalar_field_data, i, j, k);
			}
		}
	}
	std::cout << "Average value: " << sum / float(width * height * depth) << std::endl;
	scalar_field->UpdateMinMax(scalar_field_data);
	std::cout << "Max value: " << scalar_field->field_max << std::endl;
	std::cout << "Min value: " << scalar_field->field_min << std::endl;

	// Normalize data
	scalar_field->Normalize(scalar_field_data);

	//if (Approx(density, 1.7, 0.6)) {
	//	L *= Exp(-3 * density * marching_delta * sigma_a[0]);
	//} else if (Approx(density, 0.6, 0.2)) {
	//	L *= Exp(-3 * density * marching_delta * sigma_a[1]);
	//} else if (Approx(density, 1, 0.2)) {
	//	L *= Exp(-3 * density * marching_delta * sigma_a[2]);
	//}

	// Create transfer function
	TF1DControlPoint control_points[2];
	control_points[0] = TF1DControlPoint(0.f, Spectrum(1.f, 0.f, 0.f), 1.f);
	control_points[1] = TF1DControlPoint(1.f, Spectrum(1.f, 0.f, 0.f), 1.f);
	TF1D tf(2);
	TFOutput output = tf.At(control_points, 0.1f);

	// Render image
	const dim3 block(8, 32);
	const dim3 grid(DivUp(WIDTH, block.x), DivUp(HEIGHT, block.y));

	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);

	cudaEventRecord(start, 0);
	RayMarchVolume KERNEL_ARGS2(grid, block)(scalar_field, scalar_field_data, camera, film, film_raster, 0.001f);
	cudaEventRecord(stop, 0);
	// Wait for stop event
	cudaEventSynchronize(stop);
	// Print time to render
	float time;
	cudaEventElapsedTime(&time, start, stop);
	std::cout << "Computed rendering in: " << time << " ms." << std::endl;

	// Create image
	film->CreatePNG(film_raster, "render.png");

	// Free resources
	CUDA_SAFE_CALL(cudaFree(film));
	CUDA_SAFE_CALL(cudaFree(film_raster));
	CUDA_SAFE_CALL(cudaFree(camera));
	CUDA_SAFE_CALL(cudaFree(scalar_field_data));
	CUDA_SAFE_CALL(cudaFree(scalar_field));

	return 0;
}


