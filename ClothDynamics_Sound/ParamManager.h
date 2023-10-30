#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#pragma once
#include <vector>

#define USED_STREAM

using namespace std;
typedef unsigned int uint;

struct ObjectParams {
	int* faces;
	double* vertices;
	double* velocities;
	double* masses;
	uint fnum;
	uint vnum;
};

typedef vector<vector<int>> EdgeBufferHost;
typedef vector<vector<int>> NbFaceBufferHost;

struct EdgeBuffer {
	int* index;
	int* array;
};
struct NbFaceBuffer {
	int* index;
	int* array;
};

struct FaceRankParams {
	double* cens;
	double* maxs;
	double* mins;
	uint* ranks;
	uint* inds[2];
	uint* axes;
	uint* i;
};

struct ContactElem {
	bool isfv;
	uint i[4];
	double info;
};
struct ContactElemParams {
	uint h_size;
	uint* d_size;
public:
	bool* isfv;
	uint* i[4];
	double* info;
};

class ParamManager {
public:
	ParamManager() {}
	virtual ~ParamManager() {}
public:
	static inline void initObjectParamHost(ObjectParams& params, uint fnum, uint vnum, bool all) {
		params.fnum = fnum;
		params.vnum = vnum;
		params.faces = (int*)malloc(params.fnum * 3 * sizeof(int));
		params.vertices = (double*)malloc(params.vnum * 3 * sizeof(double));
		if (all) {
			params.velocities = (double*)malloc(params.vnum * 3 * sizeof(double));
			params.masses = (double*)malloc(params.vnum * sizeof(double));
		}
	}
public:
	static inline void destroyObjectParamHost(ObjectParams& params, bool all) {
		free(params.faces);
		free(params.vertices);
		if (all) {
			free(params.velocities);
			free(params.masses);
		}
	}
public:
	static inline void destroyContactElemParamHost(ContactElemParams& params) {
		free(params.i[0]);
		free(params.i[1]);
		free(params.i[2]);
		free(params.i[3]);
	}
};

#endif