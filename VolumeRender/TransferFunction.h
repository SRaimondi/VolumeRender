#ifndef TRANSFER_FUNCTION_H_
#define TRANSFER_FUNCTION_H_

#include "Spectrum.h"
#include <cassert>

// This file defines the tools to create a TransferFunction

// Transfer function output
struct TFOutput {
	// Attenuation value
	Spectrum attenuation;
	// Density valuie
	float density;

	CUDA_HOST_DEVICE inline TFOutput()
		: attenuation(), density(0.f) {
	}

	CUDA_HOST_DEVICE inline TFOutput(const Spectrum &a, float d) noexcept
		: attenuation(a), density(d) {
	}
};

CUDA_HOST_DEVICE inline TFOutput operator+(const TFOutput &t1, const TFOutput &t2) noexcept {
	return{ t1.attenuation + t2.attenuation, t1.density + t2.density };
}

CUDA_HOST_DEVICE inline TFOutput operator-(const TFOutput &t1, const TFOutput &t2) noexcept {
	return{ t1.attenuation - t2.attenuation, t1.density - t2.density };
}

CUDA_HOST_DEVICE inline TFOutput operator*(float t, const TFOutput &tf) noexcept {
	return{ t * tf.attenuation, t * tf.density };
}

CUDA_HOST_DEVICE inline TFOutput operator/(const TFOutput &tf, float t) noexcept {
	return{ tf.attenuation / t, tf.density / t };
}

// 1D control point
struct TF1DControlPoint {
	// Input value
	float value;
	// Output at this point
	TFOutput output;

	CUDA_HOST TF1DControlPoint()
		: value(0.f), output() {
	}

	CUDA_HOST_DEVICE inline TF1DControlPoint(float f_v, const Spectrum &a, float d) noexcept
		: value(f_v), output(a, d) {
	}
};

// The TransferFunction1D takes as input the scalar field value, in the range [0, 1], and outputs a density and attenuation color
class TF1D {
public:
	// Number of points in the TF
	int num_points;

	CUDA_HOST TF1D(int n_p)
		: num_points(n_p) {
		assert(num_points >= 2);
	}

	// Compute transfer function value at given parameter in [0,1]
	CUDA_HOST_DEVICE TFOutput At(const TF1DControlPoint* control_points, float x) const noexcept;
};

#endif /* TRANSFER_FUNCTION_H_ */
