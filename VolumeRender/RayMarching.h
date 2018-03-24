/*
 * RayMarching.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef RAYMARCHING_H_
#define RAYMARCHING_H_

#include "Camera.h"
#include "ScalarField.h"
#include "Film.h"

// Initial RayMarching function
CUDA_GLOBAL void RayMarchVolume(
	const ScalarFieldDescription* field_description,	// Description of the scalar field to render
	const float* field_data,							// Actual field data
	const Camera* camera,								// Camera to use
	const FilmDescription* film_description,			// Description of the film used
	Spectrum* film_raster,								// Film raster
	float marching_delta								// Step to use in the marching process
);

#endif /* RAYMARCHING_H_ */