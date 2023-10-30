#ifndef __DEVICE_MANAGER_H__
#define __DEVICE_MANAGER_H__

#pragma once
#include "device_launch_parameters.h"
#include "cuda_runtime.h"
#include <chrono>

using namespace std;
typedef unsigned int uint;
typedef unsigned char uchar;
typedef std::chrono::system_clock::time_point ctimer;

#define BLOCKSIZE			128
#define HBLOCKSIZE			64
#define WARPSIZE			32
#define EPS					1.0e-5

//#define TESTTIMER
#define CUDA_DEBUG

#ifndef CUDA_DEBUG
#define CUDA_CHECK(x)	(x)
#else
#define CUDA_CHECK(x)	do {\
		(x); \
		cudaError_t e = cudaGetLastError(); \
		if (e != cudaSuccess) { \
			printf("cuda failure %s:%d: '%s'\n", \
				__FILE__, __LINE__, cudaGetErrorString(e)); \
			/*exit(1);*/ \
		}\
	} while(0)
#endif

#define CNOW		std::chrono::system_clock::now()

#endif