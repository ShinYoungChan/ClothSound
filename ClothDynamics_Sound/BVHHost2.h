#ifndef __BVH_TREE_HOST_H__
#define __BVH_TREE_HOST_H__

#pragma once
#include "Mesh.h"
#include "Vec3.h"
#include "Lib.h"
#include "Constraint.h"
#include "ParamManager.h"

using namespace std;

class BVHNodeHost2 {
public:
	BVHNodeHost2** _childs;
	vector<int> _faces;
public:
	Vec3 _min;
	Vec3 _max;
	int _level;
	int _RTri;
public:
	BVHNodeHost2() {
		_childs = new BVHNodeHost2 * [2];
		_childs[0] = _childs[1] = nullptr;
		_min.x = _min.y = _min.z = DBL_MAX;
		_max.x = _max.y = _max.z = -DBL_MAX;
		_level = 0;
		_RTri = 63;
	}
	BVHNodeHost2(int level) {
		_childs = new BVHNodeHost2 * [2];
		_childs[0] = _childs[1] = nullptr;
		_min.x = _min.y = _min.z = DBL_MAX;
		_max.x = _max.y = _max.z = -DBL_MAX;
		_level = level;
		_RTri = 63;
	}
	virtual ~BVHNodeHost2(void)
	{
		for (int i = 0; i < 2; i++)
			if (_childs[i] != nullptr)
				delete _childs[i];
		delete[] _childs;
	}
public:
	inline bool intersect(const Vec3& p) {
		return _min.x <= p.x
			&& _min.y <= p.y
			&& _min.z <= p.z
			&& _max.x >= p.x
			&& _max.y >= p.y
			&& _max.z >= p.z;
	}
	inline bool intersect(const Vec3& pa, const Vec3& pb) {
		return _min.x <= max(pa.x, pb.x)
			&& _min.y <= max(pa.y, pb.y)
			&& _min.z <= max(pa.z, pb.z)
			&& _max.x >= min(pa.x, pb.x)
			&& _max.y >= min(pa.y, pb.y)
			&& _max.z >= min(pa.z, pb.z);
	}
	inline bool intersect(const Vec3& pa0, const Vec3& pb0, const Vec3& pc0) {
		return _min.x <= max(max(pa0.x, pb0.x), pc0.x)
			&& _min.y <= max(max(pa0.y, pb0.y), pc0.y)
			&& _min.z <= max(max(pa0.z, pb0.z), pc0.z)
			&& _max.x >= min(min(pa0.x, pb0.x), pc0.x)
			&& _max.y >= min(min(pa0.y, pb0.y), pc0.y)
			&& _max.z >= min(min(pa0.z, pb0.z), pc0.z);
	}
	inline bool intersect(const Vec3& pa0, const Vec3& pb0, const Vec3& pc0, const Vec3& pa1, const Vec3& pb1, const Vec3& pc1) {
		return _min.x <= max(max(max(max(max(pa0.x, pb0.x), pc0.x), pa1.x), pb1.x), pc1.x)
			&& _min.y <= max(max(max(max(max(pa0.y, pb0.y), pc0.y), pa1.y), pb1.y), pc1.y)
			&& _min.z <= max(max(max(max(max(pa0.z, pb0.z), pc0.z), pa1.z), pb1.z), pc1.z)
			&& _max.x >= min(min(min(min(min(pa0.x, pb0.x), pc0.x), pa1.x), pb1.x), pc1.x)
			&& _max.y >= min(min(min(min(min(pa0.y, pb0.y), pc0.y), pa1.y), pb1.y), pc1.y)
			&& _max.z >= min(min(min(min(min(pa0.z, pb0.z), pc0.z), pa1.z), pb1.z), pc1.z);
	}
	inline bool intersect(BVHNodeHost2* node) {
		return _min.x <= node->_max.x
			&& _min.y <= node->_max.y
			&& _min.z <= node->_max.z
			&& _max.x >= node->_min.x
			&& _max.y >= node->_min.y
			&& _max.z >= node->_min.z;
	}
public:
	inline void Min(void) {
		_min.x = min(min(_childs[0]->_min.x, _childs[1]->_min.x), _min.x);
		_min.y = min(min(_childs[0]->_min.y, _childs[1]->_min.y), _min.y);
		_min.z = min(min(_childs[0]->_min.z, _childs[1]->_min.z), _min.z);
	}
	inline void Max(void) {
		_max.x = max(max(_childs[0]->_max.x, _childs[1]->_max.x), _max.x);
		_max.y = max(max(_childs[0]->_max.y, _childs[1]->_max.y), _max.y);
		_max.z = max(max(_childs[0]->_max.z, _childs[1]->_max.z), _max.z);
	}
	inline void Min(const Vec3& a) {
		_min.x = min(a.x, _min.x);
		_min.y = min(a.y, _min.y);
		_min.z = min(a.z, _min.z);
	}
	inline void Max(const Vec3& a) {
		_max.x = max(a.x, _max.x);
		_max.y = max(a.y, _max.y);
		_max.z = max(a.z, _max.z);
	}
	inline void Min(const Vec3& a, const Vec3& b, const Vec3& c, double eps) {
		_min.x = min(min(min(a.x - eps, b.x - eps), c.x - eps), _min.x);
		_min.y = min(min(min(a.y - eps, b.y - eps), c.y - eps), _min.y);
		_min.z = min(min(min(a.z - eps, b.z - eps), c.z - eps), _min.z);
	}
	inline void Max(const Vec3& a, const Vec3& b, const Vec3& c, double eps) {
		_max.x = max(max(max(a.x + eps, b.x + eps), c.x + eps), _max.x);
		_max.y = max(max(max(a.y + eps, b.y + eps), c.y + eps), _max.y);
		_max.z = max(max(max(a.z + eps, b.z + eps), c.z + eps), _max.z);
	}
	void refit(const vector<int>& faces, const vector<double>& vertices, const vector<double>& velocities, double thickness, double dt, bool isCCD) {
		_min.x = _min.y = _min.z = DBL_MAX;
		_max.x = _max.y = _max.z = -DBL_MAX;
		if (_childs[0]) {
			_childs[0]->refit(faces, vertices, velocities, thickness, dt, isCCD);
			_childs[1]->refit(faces, vertices, velocities, thickness, dt, isCCD);
			Min();
			Max();
		}
		else {
			int i0 = faces[_faces[0] * 3 + 0] * 3;
			int i1 = faces[_faces[0] * 3 + 1] * 3;
			int i2 = faces[_faces[0] * 3 + 2] * 3;
			Vec3 va0(vertices[i0 + 0], vertices[i0 + 1], vertices[i0 + 2]);
			Vec3 vb0(vertices[i1 + 0], vertices[i1 + 1], vertices[i1 + 2]);
			Vec3 vc0(vertices[i2 + 0], vertices[i2 + 1], vertices[i2 + 2]);
			Min(va0, vb0, vc0, thickness);
			Max(va0, vb0, vc0, thickness);
			if (isCCD) {
				Vec3 va1(va0.x + velocities[i0 + 0] * dt, va0.y + velocities[i0 + 1] * dt, va0.z + velocities[i0 + 2] * dt);
				Vec3 vb1(vb0.x + velocities[i1 + 0] * dt, vb0.y + velocities[i1 + 1] * dt, vb0.z + velocities[i1 + 2] * dt);
				Vec3 vc1(vc0.x + velocities[i2 + 0] * dt, vc0.y + velocities[i2 + 1] * dt, vc0.z + velocities[i2 + 2] * dt);
				Min(va1, vb1, vc1, thickness);
				Max(va1, vb1, vc1, thickness);
			}
		}
	}
	void draw(void) {
		glDisable(GL_LIGHTING);
		glPushMatrix();
		glLineWidth(1.0f);
		glColor3d(0.6, 0.6, 0.6);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glBegin(GL_LINES);
		glVertex3d(_min[0], _min[1], _min[2]);
		glVertex3d(_min[0], _min[1], _max[2]);
		glVertex3d(_min[0], _max[1], _min[2]);
		glVertex3d(_min[0], _max[1], _max[2]);
		glVertex3d(_max[0], _min[1], _min[2]);
		glVertex3d(_max[0], _min[1], _max[2]);
		glVertex3d(_max[0], _max[1], _min[2]);
		glVertex3d(_max[0], _max[1], _max[2]);
		glEnd();
		glTranslated(0, 0, _min[2]);
		glRectd(_min[0], _min[1], _max[0], _max[1]);
		glTranslated(0, 0, _max[2] - _min[2]);
		glRectd(_min[0], _min[1], _max[0], _max[1]);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPopMatrix();
		glEnable(GL_LIGHTING);
	}
	inline bool RTriVertex(int n) {
		return (_RTri >> n) & 1;
	}
	inline bool RTriEdge(int n) {
		return (_RTri >> (n + 3)) & 1;
	}
	inline void RTriVertex(int n, int num) {
		if (num)
			_RTri |= 1 << n;
		else
			_RTri &= ~(1 << n);
	}
	inline void RTriEdge(int n, int num) {
		if (num)
			_RTri |= 1 << (n + 3);
		else
			_RTri &= ~(1 << (n + 3));
	}
};

class BVHHost2
{
public:
	BVHNodeHost2* _root;
	vector<int> _nfaces;
public:
	BVHHost2() { }
	BVHHost2(ObjectParams& objParams, const NbFaceBuffer& nbFaces);
	~BVHHost2() { }
public:
	void clear(void);
	void draw(void);
	void refit(const vector<int>& faces, const vector<double>& vertices, const vector<double>& velocities, double thickness, double dt, bool isCCD);
public:
	void getNearestFaces(const Vec3& p);
	void getNearestFaces(const Vec3& pa, const Vec3& pb, const Vec3& pc);
	void getNearestFaces(const Vec3& p0, const Vec3& p1);
	void getNearestFaces(const Vec3& pa0, const Vec3& pb0, const Vec3& pc0, const Vec3& pa1, const Vec3& pb1, const Vec3& pc1);
};

#endif