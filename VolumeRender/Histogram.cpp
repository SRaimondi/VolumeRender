#include "Histogram.h"
#include <iostream>

Histogram::Histogram(int size, float delta) noexcept
	: histogram_data(size, 0), size(size), delta(delta) {
}

CUDA_HOST void Histogram::Print() const noexcept {
	for (int i = 0; i < size; ++i) {
		std::cout << "[" << i * delta << " - " << (i + 1) * delta << "]:\t" << histogram_data[i] << std::endl;
		//for (int n = 0; n < histogram_data[i] / 100; ++n) {
		//	std::cout << "x";
		//}
		//std::cout << std::endl;
	}
}

CUDA_HOST Histogram GenerateHistogram(const ScalarFieldDescription * scalar_field, const float * scalar_field_data, int num_intervals) noexcept {
	Histogram hist(num_intervals, 1.f / float(num_intervals));
	// Now loop through the data and increase the counter in the histogram
	for (int i = 0; i < scalar_field->dims[0] * scalar_field->dims[1] * scalar_field->dims[2]; ++i) {
		hist.Add(Clamp(int(std::floor(scalar_field_data[i] * num_intervals)), 0, num_intervals - 1), 1);
	}

	return hist;
}
