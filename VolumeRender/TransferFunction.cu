#include "TransferFunction.h"
#include "Interpolation.h"

CUDA_HOST_DEVICE TFOutput TF1DCubic::At(const TF1DControlPoint * control_points, float x) const noexcept {
	// First, find the indices of the left point for our interval
	int left_point = 0;
	while (x > control_points[left_point + 1].value) {
		++left_point;
	}

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
	// Find the index of the closest exponential
	int exp_index = -1;
	float distance = INF;
	for (int i = 0; i < num_points; ++i) {
		const float d_temp = std::abs(x - control_points[i].value);
		if (d_temp < distance) {
			distance = d_temp;
			exp_index = i;
		}
	}

	// Evaluate exponential
	float x_b = x - control_points[exp_index].value;
	float density = control_points[exp_index].output.density * std::exp(-0.5f * x_b * x_b / variance);

	// Return value based on closest exponential
	return{ control_points[exp_index].output.attenuation, density };
}
