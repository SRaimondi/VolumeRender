#ifndef EMPTY_SPACE_MAP_H_
#define EMPTY_SPACE_MAP_H_

#include "ScalarField.h"

// Utility class that processes a ScalarFieldDescription to set voxels we consider "empty"
class EmptySpaceMap {
public:
	template <typename TF_FUNC>
	static void ProcessScalarField(const ScalarFieldDescription* field, const float* data,
								   const TF_FUNC* transfer_function,
								   const TF1DControlPoint* tf_control_points,
								   bool* empty_space_map,
								   float tollerance) noexcept;
};

// This method could be improved by precomputing the TF value at each vertex and then loop through them
// We assume the data has been normalized
template <typename TF_FUNC>
void EmptySpaceMap::ProcessScalarField(const ScalarFieldDescription* field, const float* data,
									   const TF_FUNC* transfer_function,
									   const TF1DControlPoint* tf_control_points,
									   bool* empty_space_map,
									   float tollerance) noexcept {
	// Loop over all voxels in the scalar field
	for (int k = 0; k < field->dims[2] - 1; ++k) {
		for (int j = 0; j < field->dims[1] - 1; ++j) {
			for (int i = 0; i < field->dims[0] - 1; ++i) {
				// Evaluate the transfer function at each vertex of the voxel
				float tf_vertices[8];
				for (int v_k = 0; v_k < 2; ++v_k) {
					for (int v_j = 0; v_j < 2; ++v_j) {
						for (int v_i = 0; v_i < 2; ++v_i) {
							tf_vertices[v_i + v_j * 2 + v_k * 4] = transfer_function->At(tf_control_points, (*field)(data, i + v_i, j + v_j, k + v_k)).density;
								/* field->is_normalized ?
								// Evaluate transfer function directly
								transfer_function->At(tf_control_points, (*field)(data, i + v_i, j + v_j, k + v_k)).density :
								// Evaluate transfer function after normalizing value
								transfer_function->At(tf_control_points, field->NormalizeVal((*field)(data, i + v_i, j + v_j, k + v_k))).density; */
						}
					}
				}
				bool is_empty = true;
				for (int vertex = 0; vertex < 8; ++vertex) {
					if (tf_vertices[vertex] > tollerance) {
						// Space is not empty
						is_empty = false;
						break;
					}
				}
				empty_space_map[field->Offset(i, j, k)] = is_empty;
			}
		}
	}
}

#endif /* EMPTY_SPACE_MAP_H_ */
