/*
 * Interpolation.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef INTERPOLATION_H_
#define INTERPOLATION_H_

#include "Common.h"

// Lerp
template <typename T>
CUDA_HOST_DEVICE inline static T Lerp(float t, T a, T b) noexcept {
	return (1.f - t) * a + t * b;
}

// Cubic interpolation
template <typename T>
CUDA_HOST_DEVICE inline static T CINT_CR(float t, T u0, T u1, T u2, T u3) noexcept {
	// Evaluate coefficients for points
	float t2 = t * t;
	float d0 = t2 * (2.f - t) - t;
	float d1 = t2 * (3.f * t - 5.f) + 2.f;
	float d2 = t2 * (4.f - 3.f * t) + t;
	float d3 = t2 * (t - 1.f);

	return 0.5f * (d0 * u0 + d1 * u1 + d2 * u2 + d3 * u3);
}

// Cubic interpolation for arbitrary interval
template <typename T>
CUDA_HOST_DEVICE inline static T CubicSpline(float x, float xk, float xk_1,
											 T pk, T mk, T pk_1, T mk_1) noexcept {
	// Compute t
	float delta = xk_1 - xk;
	float t = FMax(x - xk, 0.f) / delta;
	float t2 = t * t;
	float t3 = t2 * t;
	// Compute basis function value
	float h00 = 2.f * t3 - 3.f * t2 + 1.f;
	float h10 = t3 - 2.f * t2 + t;
	float h01 = -2.f * t3 + 3.f * t2;
	float h11 = t3 - t2;

	return h00 * pk + h10 * delta * mk + h01 * pk_1 + h11 * delta * mk_1;
}

#endif /* INTERPOLATION_H_ */
