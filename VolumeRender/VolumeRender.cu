#include "DDSLoader.h"
#include "RayMarching.h"
#include "TransferFunction.h"
#include "EmptySpaceMap.h"

#include <iostream>

int main(void) {
	// Create film
	constexpr int WIDTH = 1024;
	constexpr int HEIGHT = 1024;
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

	TF1DControlPoint* tf_control_points;
	const Spectrum sigma_a[3] = {
		Spectrum(1) - Spectrum(0.1, 0.7, 0.1),
		Spectrum(1) - Spectrum(0.2, 0.3, 0.7),
		Spectrum(1) - Spectrum(0.1, 0.1, 0.5) };


	// Create cubic transfer function
	//constexpr int NUM_CNTRL_POINTS = 5;
	//CUDA_SAFE_CALL(cudaMallocManaged(&tf_control_points, NUM_CNTRL_POINTS * sizeof(TF1DControlPoint)));
	//tf_control_points[0] = TF1DControlPoint(0.f, Spectrum(0.f, 0.6f, 0.6f), 0.5f);
	//tf_control_points[1] = TF1DControlPoint(0.06f, sigma_a[1], 1.f);
	//tf_control_points[2] = TF1DControlPoint(0.1f, sigma_a[2], 1.f);
	//tf_control_points[3] = TF1DControlPoint(0.17f, sigma_a[0], 2.f);
	//tf_control_points[4] = TF1DControlPoint(1.f, Spectrum(0.f, 0.6f, 0.6f), 0.5f);
	//
	//TF1DCubic* tf;
	//CUDA_SAFE_CALL(cudaMallocManaged(&tf, sizeof(TF1DCubic)));
	//tf->num_points = NUM_CNTRL_POINTS;

	// Create exponential transfer function
	constexpr int NUM_CNTRL_POINTS = 3;
	CUDA_SAFE_CALL(cudaMallocManaged(&tf_control_points, NUM_CNTRL_POINTS * sizeof(TF1DControlPoint)));
	tf_control_points[0] = TF1DControlPoint(0.06f, sigma_a[1], 4.f);
	tf_control_points[1] = TF1DControlPoint(0.1f, sigma_a[2], 4.f);
	tf_control_points[2] = TF1DControlPoint(0.17f, sigma_a[0], 4.f);

	TF1DExp* tf;
	CUDA_SAFE_CALL(cudaMallocManaged(&tf, sizeof(TF1DExp)));
	tf->num_points = NUM_CNTRL_POINTS;
	tf->variance = 0.00001f;


	// Compute empty space map
	bool* empty_space_map;
	CUDA_SAFE_CALL(cudaMallocManaged(&empty_space_map, (width - 1) * (height - 1) * (depth - 1) * sizeof(bool)));
	EmptySpaceMap::ProcessScalarField(scalar_field, scalar_field_data,
									  tf, tf_control_points,
									  empty_space_map, 0.01f);


	// Render image
	const dim3 block(16, 16);
	const dim3 grid(DivUp(WIDTH, block.x), DivUp(HEIGHT, block.y));

	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);

	constexpr float MARCHING_STEP = 0.001f;
	cudaEventRecord(start, 0);
	//RayMarchVolume KERNEL_ARGS2(grid, block)(scalar_field, scalar_field_data,
	//										 camera, 
	//										 film, film_raster,
	//										 MARCHING_STEP,
	//										 tf, tf_control_points);

	RayMarchVolumeEmptySpaceMap KERNEL_ARGS2(grid, block) (scalar_field, scalar_field_data,
														   empty_space_map,
														   camera,
														   film, film_raster,
														   MARCHING_STEP,
														   tf, tf_control_points);

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
	CUDA_SAFE_CALL(cudaFree(empty_space_map));
	CUDA_SAFE_CALL(cudaFree(tf));
	CUDA_SAFE_CALL(cudaFree(tf_control_points));
	CUDA_SAFE_CALL(cudaFree(film));
	CUDA_SAFE_CALL(cudaFree(film_raster));
	CUDA_SAFE_CALL(cudaFree(camera));
	CUDA_SAFE_CALL(cudaFree(scalar_field_data));
	CUDA_SAFE_CALL(cudaFree(scalar_field));

	return 0;
}


