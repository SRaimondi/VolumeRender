/*
 * Spectrum.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef SPECTRUM_H_
#define SPECTRUM_H_

#include "Common.h"

class Spectrum {
public:
	// Color components
	float r, g, b;

	// Constructors
	CUDA_HOST_DEVICE inline constexpr Spectrum() noexcept
		: r(0), g(0), b(0) {
	}

	CUDA_HOST_DEVICE inline explicit Spectrum(float v) noexcept
		: r(v), g(v), b(v) {
	}

	CUDA_HOST_DEVICE inline Spectrum(float r, float g, float b) noexcept
		: r(r), g(g), b(b) {
	}

	CUDA_HOST_DEVICE inline Spectrum &operator*=(const Spectrum &s) noexcept {
		r *= s.r;
		g *= s.g;
		b *= s.b;
		return *this;
	}

	CUDA_HOST_DEVICE inline Spectrum &operator*=(float t) noexcept {
		r *= t;
		g *= t;
		b *= t;
		return *this;
	}
};

// Math operators
CUDA_HOST_DEVICE inline Spectrum operator+(const Spectrum &s1, const Spectrum &s2) noexcept {
	return{ s1.r + s2.r, s1.g + s2.g, s1.b + s2.b };
}

CUDA_HOST_DEVICE inline Spectrum operator-(const Spectrum &s1, const Spectrum &s2) noexcept {
	return{ s1.r - s2.r, s1.g - s2.g, s1.b - s2.b };
}

CUDA_HOST_DEVICE inline Spectrum operator*(const Spectrum &s1, const Spectrum &s2) noexcept {
	return{ s1.r * s2.r, s1.g * s2.g, s1.b * s2.b };
}

CUDA_HOST_DEVICE inline Spectrum operator*(float t, const Spectrum &s) noexcept {
	return{ t * s.r, t * s.g, t * s.b };
}

CUDA_HOST_DEVICE inline Spectrum operator*(const Spectrum &s, float t) noexcept {
	return{ t * s.r, t * s.g, t * s.b };
}

CUDA_HOST_DEVICE inline Spectrum operator/(const Spectrum &s, float t) noexcept {
	return{ s.r / t, s.g / t, s.b / t };
}

// Clamp Spectrum
CUDA_HOST_DEVICE inline Spectrum Clamp(const Spectrum &s, float min, float max) noexcept {
	return{ ::Clamp(s.r, min, max),
		::Clamp(s.g, min, max),
		::Clamp(s.b, min, max) };
}

// Spectrum exponential
CUDA_HOST_DEVICE inline Spectrum Exp(const Spectrum &s) noexcept {
	return{ std::exp(s.r), std::exp(s.g), std::exp(s.b) };
}

// Spectrum log
CUDA_HOST_DEVICE inline Spectrum Log(const Spectrum &s) noexcept {
	return{ std::log(s.r), std::log(s.g), std::log(s.b) };
}

#endif /* SPECTRUM_H_ */
