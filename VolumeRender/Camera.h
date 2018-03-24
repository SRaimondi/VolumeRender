/*
 * Camera.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "Geometry.h"

class Camera {
private:
	// Camera position
	Vec3 eye;
	// Local camera base
	Vec3 u, v, w;
	// View plane distance
	float dist;
	// View-port
	float b, t, l, r;
	// Target film size
	int width, height;

public:
	CUDA_HOST Camera(const Vec3 &e, const Vec3 &at, const Vec3 &up,
					 float fov, float d, int width, int height) noexcept;

	// Generate Ray given pixel position and offset
	CUDA_HOST_DEVICE Ray inline GenerateRay(int i, int j, float sx, float sy) const noexcept {
		// Compute view plane coordinates
		float vp_x = l + (r - l) * (i + sx) / float(width);
		float vp_y = t - (t - b) * (j + sy) / float(height);
		// Compute point on view plane
		Vec3 s = vp_x * u + vp_y * v - dist * w;

		return{ eye, Normalize(s) };
	}

	// Update the width and height of the view-port
	CUDA_HOST void UpdateViewPort(int new_w, int new_h) noexcept;

	// Update camera position using spherical coordinates deltas
	// CUDA_HOST void UpdateCameraSC(float d_radius, float d_phi, float d_theta) noexcept;
};

#endif /* CAMERA_H_ */
