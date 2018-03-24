#include "Film.h"
#include "LodePng.h"
#include <vector>
#include <iostream>

CUDA_HOST FilmDescription::FilmDescription(int w, int h) noexcept
	: width(w), height(h) {
}


CUDA_HOST void FilmDescription::CreatePNG(const Spectrum* raster, const std::string &file_name) const {
	// Prepare data
	std::vector<unsigned char> image;
	image.reserve(width * height * 4);

	for (int j = 0; j < height; ++j) {
		for (int i = 0; i < width; ++i) {
			const Spectrum c = Clamp(this->operator()(raster, i, j), 0.f, 1.f);
			image.emplace_back(static_cast<unsigned char> (c.r * 255));
			image.emplace_back(static_cast<unsigned char> (c.g * 255));
			image.emplace_back(static_cast<unsigned char> (c.b * 255));
			// Alpha
			image.emplace_back(static_cast<unsigned char> (255));
		}
	}

	// Encode the image
	const unsigned int error = lodepng::encode(file_name, image, width, height);
	if (error) {
		std::cerr << "Encode error: " << error << ": " <<
			lodepng_error_text(error) << std::endl;
	}
}
