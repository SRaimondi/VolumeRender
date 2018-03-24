#include "EmptySpaceMap.h"

CUDA_HOST void EmptySpaceMap::ProcessScalarField(const ScalarFieldDescription * field, const float * data, 
												 bool* empty_space_map, 
												 float tollerance) noexcept {
	// Loop over all voxels in the scalar field
	for (int k = 0; k < field->dims[2] - 1; ++k) {
		for (int j = 0; j < field->dims[1] - 1; ++j) {
			for (int i = 0; i < field->dims[0] - 1; ++i) {
				// Check if all the 8 corners are below tollerance
				empty_space_map[field->Offset(i, j, k)] =
					// Back face
					(*field)(data, i, j, k) < tollerance &&
					(*field)(data, i + 1, j, k) < tollerance &&
					(*field)(data, i, j + 1, k) < tollerance &&
					(*field)(data, i + 1, j + 1, k) < tollerance &&
					// Front face
					(*field)(data, i, j, k + 1) < tollerance &&
					(*field)(data, i + 1, j, k + 1) < tollerance &&
					(*field)(data, i, j + 1, k + 1) < tollerance &&
					(*field)(data, i + 1, j + 1, k + 1) < tollerance;
			}
		}
	}
}
