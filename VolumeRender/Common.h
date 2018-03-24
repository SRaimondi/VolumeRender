/*
 * Common.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <limits>
#include <algorithm>
#include <cstdio>

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <device_functions.h>

// Define marco for host and device code
#ifdef __CUDACC__
#define CUDA_HOST_DEVICE __host__ __device__
#define CUDA_HOST __host__
#define CUDA_DEVICE __device__
#define CUDA_GLOBAL __global__
#else
#define CUDA_HOST_DEVICE
#define CUDA_HOST
#define CUDA_DEVICE
#define CUDA_GLOBAL
#endif

// Define constants used in project
constexpr float PI = 3.14159265358979323846f;
constexpr float INF = std::numeric_limits<float>::max();
constexpr float NEG_INF = std::numeric_limits<float>::lowest();
constexpr float TOL = 10e-5f;

// Error check macro
#ifndef NDEBUG
#define CUDA_SAFE_CALL(call) \
{ \
const cudaError_t error = call; \
if (error != cudaSuccess) { \
fprintf(stderr, " %s: %d ", __FILE__, __LINE__); \
fprintf(stderr, " Code: %d, reason: %s\n", error, cudaGetErrorString(error)); \
exit(EXIT_FAILURE);	\
} \
}
#else
#define CUDA_SAFE_CALL(call) \
{ \
call; \
}
#endif

// Define MACROS for kernel arguments
#ifdef __CUDACC__
#define KERNEL_ARGS2(grid, block) <<< grid, block >>>
#define KERNEL_ARGS3(grid, block, sh_mem) <<< grid, block, sh_mem >>>
#define KERNEL_ARGS4(grid, block, sh_mem, stream) <<< grid, block, sh_mem, stream >>>
#else
#define KERNEL_ARGS2(grid, block)
#define KERNEL_ARGS3(grid, block, sh_mem)
#define KERNEL_ARGS4(grid, block, sh_mem, stream)
#endif

// Convert to radians
CUDA_HOST_DEVICE inline static float Radians(float deg) noexcept {
	return deg / 180.f * PI;
}

// Clamp value
template <typename T>
CUDA_HOST_DEVICE inline static T Clamp(T v, T min, T max) noexcept {
	return (v < min ? min : (v > max ? max : v));
}

// Divide up to closest multiple
CUDA_HOST_DEVICE inline static int DivUp(int a, int b) {
	return (a % b != 0) ? (int(a / b) + 1) : (a / b);
}

// Divide down to closest multiple
CUDA_HOST_DEVICE inline static int DivDown(int a, int b) {
	return a / b;
}

// Fast min and max
template <typename T>
CUDA_HOST_DEVICE inline static T FMin(T a, T b) noexcept {
	return a < b ? a : b;
}

template <typename T>
CUDA_HOST_DEVICE inline static T FMax(T a, T b) noexcept {
	return a > b ? a : b;
}

// Check if two values are closer than a given tolerance
CUDA_HOST_DEVICE inline static bool Approx(float a, float b, float tol = TOL) noexcept {
	return std::abs(a - b) <= tol;
}

// Swap two values
template <typename T>
CUDA_HOST_DEVICE inline static void Swap(T &a, T &b) noexcept {
	T c = a;
	a = b;
	b = c;
}


#endif /* COMMON_H_ */
