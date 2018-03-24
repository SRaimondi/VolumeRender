#ifndef HISTOGRAM_H_
#define HISTOGRAM_H_

#include "ScalarField.h"
#include <cassert>
#include <vector>

// Creates an histogram of a given Scalar field
class Histogram {
private:
	// Histogram values
	std::vector<int> histogram_data;
	// Size of the histogram
	int size;
	// Interval width
	float delta;

public:
	CUDA_HOST explicit Histogram(int size, float delta) noexcept;

	// Add element to given point
	CUDA_HOST inline void Add(int index, int num_elements) noexcept {
		histogram_data[index] += num_elements;
	}

	// Get frequency at given point
	CUDA_HOST inline int Frequency(int index) const noexcept {
		assert(index >= 0 && index < size);
		return histogram_data[index];
	}

	// Print out histogram to given ostream
	CUDA_HOST void Print() const noexcept;
};

// Generate Histogram from a scalar field, supposed to be normalized
CUDA_HOST Histogram GenerateHistogram(const ScalarFieldDescription* scalar_field, const float* scalar_field_data, int num_intervals) noexcept;

#endif /* HISTOGRAM_H_ */
