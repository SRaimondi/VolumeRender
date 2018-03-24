/*
 * Film.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef FILM_H_
#define FILM_H_

#include "Spectrum.h"
#include <memory>
#include <string>

class FilmDescription {
private:
	// Film size
	int width, height;

public:
	CUDA_HOST FilmDescription(int w, int h) noexcept;

	// Access film width and height
	CUDA_HOST_DEVICE inline int Width() const noexcept {
		return width;
	}

	CUDA_HOST_DEVICE inline int Height() const noexcept {
		return height;
	}

	// Get pixel at given indices
	CUDA_HOST_DEVICE inline const Spectrum &operator()(const Spectrum* raster, int i, int j) const noexcept {
		return raster[j * width + i];
	}

	CUDA_HOST_DEVICE inline void Set(Spectrum* raster, const Spectrum &s, int i, int j) const noexcept {
		raster[j * width + i] = s;
	}

	// Create PNG image
	CUDA_HOST void CreatePNG(const Spectrum* raster, const std::string &file_name) const;
};

#endif /* FILM_H_ */
