#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#pragma once
#include "Constraint.h"
//#include "CollisionSolver.h"
#include "CollisionSolverHost.h"

class Obstacle {
public:
	ObjectParams	h_objParams;
	NbFaceBuffer	h_nbFaces;
	//BVH*			_bvh;
	glm::dvec3		_pos;
	double			_thickness;
	double			_size;
	double			_w;
	BVHHost*		_bvhHost;
	vector<double>	_pos0;
	vector<double>	_masses;
public:
	Mesh* _mesh;
public:
	Eigen::MatrixXd		_nodes;
	Eigen::MatrixXi		_faces;
	Eigen::VectorXd		_ms;
	Eigen::MatrixX3d	_velocities;
	Eigen::MatrixXf		_normals;
	vector<bool>		_fixedPoints;
public:
	double				_zv;
	double				_za;
public:
	Obstacle(void);
	Obstacle(Mesh* mesh, double x, double y, double z, double w, double size, bool apply_scale = true);
	~Obstacle(void);
public:
	inline float			m(int i) { return _ms(i); }
	inline int				fv(int i, int j) { return _faces.row(i)[j]; }
	inline Vec3				p(int i) { return Vec3(_nodes.row(i)[0], _nodes.row(i)[1], _nodes.row(i)[2]); }
	void					p(int i, Vec3 pos) {
		_nodes.row(i)[0] = pos.x;
		_nodes.row(i)[1] = pos.y;
		_nodes.row(i)[2] = pos.z;
	}
	inline Vec3		v(int i) { return Vec3(_velocities.row(i).x(), _velocities.row(i).y(), _velocities.row(i).z()); }
	void					v(int i, Vec3 vel)
	{
		_velocities.row(i).x() = vel.x;
		_velocities.row(i).y() = vel.y;
		_velocities.row(i).z() = vel.z;
	}
	inline bool				isFixed(int i) { return _fixedPoints[i]; }
public:
	void scale(double size);
	void init(bool apply_scale = true);
	void initBVH(void);
	void update(double dt);
public:
	void computeNormal(void);
public:
	void draw(void);
	void drawWire(void);
	void drawSurface(void);
};

#endif