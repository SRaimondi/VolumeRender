#include "TransferFunction.h"
#include "Interpolation.h"

CUDA_HOST_DEVICE TFOutput TF1DCubic::At(const TF1DControlPoint * control_points, float x) const noexcept {
	// First, find the indices of the left point for our interval
	int left_point = 0;
	while (x > control_points[left_point + 1].value) {
		++left_point;
	}
	// Check we did not go after the last value
	left_point = Clamp(left_point, 0, num_points - 2);

	// Compute derivatives based on where we are
	TFOutput mk, mk_1;
	// Check if we are at the left boundary
	if (left_point == 0) {
		mk = (control_points[left_point + 1].output - control_points[left_point].output) /
			(control_points[left_point + 1].value - control_points[left_point].value);
	} else {
		mk = 0.5f *
			((control_points[left_point + 1].output - control_points[left_point].output) /
			(control_points[left_point + 1].value - control_points[left_point].value) +
			 (control_points[left_point].output - control_points[left_point - 1].output) /
			 (control_points[left_point].value - control_points[left_point - 1].value));
	}
	// Check if we are at the right boundary
	if (left_point + 1 == num_points - 1) {
		mk_1 = (control_points[left_point + 1].output - control_points[left_point].output) /
			(control_points[left_point + 1].value - control_points[left_point].value);
	} else {
		mk_1 = 0.5f *
			((control_points[left_point + 2].output - control_points[left_point + 1].output) /
			(control_points[left_point + 2].value - control_points[left_point + 1].value) +
			 (control_points[left_point + 1].output - control_points[left_point].output) /
			 (control_points[left_point + 1].value - control_points[left_point].value));
	}

	// Interpoalte using cubic spline
	return CubicSpline(x, control_points[left_point].value, control_points[left_point + 1].value,
					   control_points[left_point].output, mk, control_points[left_point + 1].output, mk_1);
}

CUDA_HOST_DEVICE TFOutput TF1DExp::At(const TF1DControlPoint * control_points, float x) const noexcept {
	// We evaluate our value as a sum of gaussians
	Spectrum attenuation;
	for (int peek = 0; peek < num_points; ++peek) {
		float x_b = x - control_points[peek].value;
		float d = control_points[peek].output.density * std::exp(-0.5f * x_b * x_b / control_points[peek].variance);
		attenuation += d * control_points[peek].output.attenuation;
	}

	return{ attenuation, 1.f };
}
