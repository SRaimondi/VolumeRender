#include "Camera.h"

CUDA_HOST Camera::Camera(const Vec3 &e, const Vec3 &at, const Vec3 &up,
						 float fov, float d, int width, int height) noexcept
	: eye(e), dist(d), width(width), height(height) {
	// Compute right and left window
	r = std::tan(Radians(fov / 2.f));
	l = -r;
	// Compute top and bottom of view port
	t = (height / float(width)) * r;
	b = -t;
	// Compute local base
	w = Normalize(e - at);
	u = Normalize(Cross(up, w));
	v = Cross(w, u);
}

CUDA_HOST void Camera::UpdateViewPort(int new_w, int new_h) noexcept {
	// Update windows size
	width = new_w;
	height = new_h;
	// Update top and bottom
	t = (height / float(width)) * r;
	b = -t;
}

//CUDA_HOST void UpdateCameraSC(float d_radius, float d_phi, float d_theta) noexcept {
//	// Compute
//}
