#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__

#pragma once
#include "Lib.h"
#include "Mesh.h"
#include "ParamManager.h"

#define BENDING

//#define DEBUG_SPRING

using namespace std;

class Spring
{
public:
	int		_ps[2];
	int		_cs[2];
	int		_fs[2];
public:
	Vec3	_vec;
public:
	double	_length;
	double	_vlength;
	double	_stiffness;
	double	_lambda; // Lagrange multiplier
	double	_lambda2; // Lagrange multiplier
	double	_tearTheta;
public:
	bool	_flag;
public:
	Spring(double stiffness, int p0, int p1, int c0, int c1, int f0, int f1, double length, double vlength, const Vec3& vec, double teartheta = -1.0)
	{
		_ps[0] = p0;
		_ps[1] = p1;
		_cs[0] = c0;
		_cs[1] = c1;
		_fs[0] = f0;
		_fs[1] = f1;
		_lambda = 0.0;
		_length = length;
		_vlength = vlength;
		_vec = vec;
		_stiffness = stiffness;
		if (teartheta > 0.0)
			_tearTheta = teartheta;
		else
			_tearTheta = (double)rand() / (double)RAND_MAX * 1.0 + 1.5;
		_flag = true;
	}
public:
	inline int modifiyVertex(int prev, int i) {
#ifdef DEBUG_SPRING
		if (_ps[0] != prev && _ps[1] != prev)
			printf("Error Spring::modifiyVertex, %d, %d, %d\n", _ps[0], _ps[1], prev);
#endif
		if (_ps[0] == prev) {
			modifiyVertexID(0, i);
			return 0;
		}
		modifiyVertexID(1, i);
		return 1;
	}
	inline void modifiyVertexID(int index, int i) {
		_ps[index] = i;
	}
	inline void modifiyCrossV(int prev, int i) {
#ifdef DEBUG_SPRING
		if (_cs[0] != prev && _cs[1] != prev)
			printf("Error Spring::modifiyCrossV, %d, %d, %d\n", _cs[0], _cs[1], prev);
#endif
		if (_cs[0] == prev)
			_cs[0] = i;
		else
			_cs[1] = i;
	}
	inline void modifiyNbFace(int prev, int i) {
#ifdef DEBUG_SPRING
		if (_fs[0] != prev && _fs[1] != prev)
			printf("Error Spring::modifiyNbFace, %d, %d, %d\n", _fs[0], _fs[1], prev);
#endif
		if (_fs[0] == prev)
			_fs[0] = i;
		else// if (_fs[1] == prev)
			_fs[1] = i;
	}
	inline void deleteNbFace(int i) {
#ifdef DEBUG_SPRING
		if (_fs[0] != i && _fs[1] != i)
			printf("Error Spring::deleteNbFace, %d, %d, %d\n", _fs[0], _fs[1], i);
#endif
		if (_fs[0] == i) {
			_fs[0] = _fs[1];
			_cs[0] = _cs[1];
		}
		_fs[1] = -1;
		_cs[1] = -1;
	}
	inline Vec3 getVecID(int i) {
#ifdef DEBUG_SPRING
		if (_ps[0] != i && _ps[1] != i)
			printf("Error Spring::getVecID, %d, %d, %d\n", _ps[0], _ps[1], i);
#endif
		if (_ps[0] == i) {
			return _vec;
		}
		return -_vec;
	}
	inline int getOtherNbFace(int i) {
#ifdef DEBUG_SPRING
		if (_fs[0] != i && _fs[1] != i)
			printf("Error Spring::getOtherNbFace, %d, %d, %d\n", _fs[0], _fs[1], i);
#endif
		if (_fs[0] == i) {
			return _fs[1];
		}
		return _fs[0];
	}
	inline void updateVlen(VectorXi& triangles, vector<Spring*>& springs) {
		if (_fs[1] > -1) {
			Vec3 v;
			for (int i = 0; i < 3; i++) {
				auto s = springs[triangles[_fs[0] * 3 + i]];
				if (s == this) continue;
				if (s->_ps[0] == _ps[0]) {
					v = s->_vec;
					break;
				} 
				else if (s->_ps[1] == _ps[0]) {
					v = -s->_vec;
					break;
				}
			}
			for (int i = 0; i < 3; i++) {
				auto s = springs[triangles[_fs[1] * 3 + i]];
				if (s == this) continue;
				if (s->_ps[0] == _ps[0]) {
					v -= s->_vec;
					break;
				}
				else if (s->_ps[1] == _ps[0]) {
					v += s->_vec;
					break;
				}
			}
			_vlength = v.Length();
		}
		else {
			Vec3 v;
			for (int i = 0; i < 3; i++) {
				auto s = springs[triangles[_fs[0] * 3 + i]];
				if (s == this) continue;
				if (s->_ps[0] == _ps[0]) {
					v = s->_vec;
					break;
				} if (s->_ps[1] == _ps[0]) {
					v = -s->_vec;
					break;
				}
			}
			_vlength = (_vec * 0.5 - v).Length();
		}
	}
};

class TearingBuffer {
public:
	int inodes[4];
	int isprings[5];
	int inodes_ord[2][3];
	int isprings_ord[2][3];
public:
	TearingBuffer() {}
	~TearingBuffer() {}
public:
	void getOrders(VectorXi &faces, VectorXi &triangles, vector<Spring*>& springs, int i)
	{
		inodes[0] = inodes[1] = inodes[2] = inodes[3] = -1;
		isprings[0] = isprings[1] = isprings[2] = isprings[3] = isprings[4] = -1;
		inodes_ord[0][0] = inodes_ord[0][1] = inodes_ord[0][2] = -1;
		inodes_ord[1][0] = inodes_ord[1][1] = inodes_ord[1][2] = -1;
		isprings_ord[0][0] = isprings_ord[0][1] = isprings_ord[0][2] = -1;
		isprings_ord[1][0] = isprings_ord[1][1] = isprings_ord[1][2] = -1;

		auto spring = springs[i];
		int i0 = spring->_ps[0];
		int i1 = spring->_ps[1];
		int if0 = spring->_fs[0];
		int if1 = spring->_fs[1];
		inodes[0] = i0;
		inodes[1] = spring->_cs[0];
		inodes[2] = i1;
		isprings[0] = i;
		for (int j = 0; j < 3; j++) {
			int ino = faces[if0 * 3 + j];
			if (ino == i0)		inodes_ord[0][0] = j;
			else if (ino == i1) inodes_ord[0][1] = j;
			else				inodes_ord[0][2] = j; 

			ino = triangles[if0 * 3 + j];
			if (springs[ino] == spring) isprings_ord[0][0] = j;
			else if (springs[ino]->_ps[0] == i0 || springs[ino]->_ps[1] == i0)
			{
				isprings[1] = ino;
				isprings_ord[0][1] = j;
			}
			else {
				isprings[2] = ino;
				isprings_ord[0][2] = j;
			}
		}
		if (if1 > 0) {
			inodes[3] = spring->_cs[1];
			for (int j = 0; j < 3; j++) {
				int ino = faces[if1 * 3 + j];
				if (ino == i0)		inodes_ord[1][0] = j;
				else if (ino == i1) inodes_ord[1][1] = j;
				else				inodes_ord[1][2] = j;

				ino = triangles[if1 * 3 + j];
				if (springs[ino] == spring) isprings_ord[1][0] = j;
				else if (springs[ino]->_ps[0] == i0 || springs[ino]->_ps[1] == i0)
				{
					isprings[4] = ino;
					isprings_ord[1][1] = j;
				}
				else {
					isprings[3] = ino;
					isprings_ord[1][2] = j;
				}
			}
		}
	}
};

class Constraint
{
public:
	VectorXi			_triangles;
	vector<Spring*>		_springs;
public:
	vector<Vec3>		_testPoint;
public:
	Constraint();
	~Constraint();
public:
	void	init(Mesh* mesh, VectorXi &faces);
	void	clear(void);
public:
	void	project(VectorXd &x, SparseMatrixXd &inv_masses, int iter);
	void	projectXPBD(VectorXd &x, SparseMatrixXd &inv_masses, double dt, int iter);
public:
	inline void	addEdge(EdgeBufferHost& edges, int p0, int p1);
	inline void	delEdge(EdgeBufferHost& edges, int p0, int p1);
public:
	int		addNode(VectorXd &x, VectorXd &px, vector<double>& masses, EdgeBufferHost& edges, const Vec3& pos, const Vec3& ppos);
	void	addSpring(EdgeBufferHost& edges, int p0, int p1, int v0, int v1, int f0, int f1, double length, double vlength, Vec3 vec, double teartheta);
	int		addFace(VectorXi &faces, int n0, int n1, int n2, int s0, int s1, int s2, int* inodes_ord, int* isprings_ord);
public:
	int		getAdjNum(Spring* sbegin, Spring* send, int pastFace, int pivot);
	void	adjustVertex(VectorXi& faces, EdgeBufferHost& edges, Spring* spring, int pastFace, int pivot, int np);
	void	adjustVertex(VectorXi& faces, EdgeBufferHost& edges, Spring* spring, int pivot, int np);
	void	cleanup(
		VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
		Spring* spring, int pivot);
public:
	void	splitFace(VectorXi &faces, vector<double>& masses, EdgeBufferHost& edges, Spring* spring, int np, double w, TearingBuffer* order);
	void	cutFace(
		VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
		Spring* spring, int np0, int np1, double w, TearingBuffer* order);
public:
	void	sliceHori(
		VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
		Spring* spring, double threshold, TearingBuffer* order);
	void	sliceHori(
		VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
		int ispring, double threshold);
	void	sliceVert(
		VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
		Spring* spring, double threshold, TearingBuffer* order);
	void	sliceVert(
		VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
		int ispring, double threshold);
public:
	bool	getSliceRatio(
		const Vec3& grad, const Vec3* ps, double threshold,
		const TearingBuffer* order, int* regions, double* ratio);
	bool	getSliceRatio3(
		const Vec3& grad, const Vec3* ps, double threshold,
		const TearingBuffer* order, int* regions, double* ratio);
	double	weaklyVertex(VectorXd& x, Spring* spring0, Spring* spring1, int pastFace0, int pastFace1, int pivot, const Vec3& grad);
	void	tearing(VectorXi &faces, VectorXd &x, VectorXd &px, vector<double>& masses, EdgeBufferHost& edges);
};

#endif