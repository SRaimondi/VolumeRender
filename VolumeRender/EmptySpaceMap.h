#ifndef EMPTY_SPACE_MAP_H_
#define EMPTY_SPACE_MAP_H_

#include "ScalarField.h"

// Utility class that processes a ScalarFieldDescription to set voxels we consider "empty"
class EmptySpaceMap {
public:
	CUDA_HOST static void ProcessScalarField(const ScalarFieldDescription* field, const float* data,
											 bool* empty_space_map,
											 float tollerance) noexcept;
};

#endif /* EMPTY_SPACE_MAP_H_ */
