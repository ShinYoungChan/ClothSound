#ifndef __CLOTH_DYNAMICS_PBD_H__
#define __CLOTH_DYNAMICS_PBD_H__

#pragma once
#include <cmath>
#include "Obstacle.h"

//#define USED_COLLISION_HOST
//#define PBD_BENDING

//struct EdgeBuffer {
//	vector<int>	_array;
//	vector<int>	_index;
//};
class ClothDynamics_PBD
{
public:
	int		_type;
	int		_maxIter;
	int		_numNodes;
	double	_dt;
	double	_timer;
	double	_mass;
	Mesh	*_mesh;
public:
	SparseMatrixXd		_M;
	SparseMatrixXd		_invMasses;
	VectorXd			_force;
	VectorXd			_nodes;
	VectorXi			_faces;
	VectorXd			_velocities;
	MatrixXd			_normals;
	MatrixXd			_nnormals;
	Constraint			*_constraint;
	vector<int>			_fixedPoints;
	int					_fixedPoints0;
public:
	EdgeBufferHost		_edges;
	NbFaceBufferHost	_nbFaces;
	int					_clickID;
	vector<Vec3>		_testPoints;
public:
	Vec3					_receiver;
	int						_w;
	int						_gw;
	vector<float>			_gkernel;
	vector<float>			_skernel;
	vector<float>			_buffer;
	vector<vector<float>>	_prevS;
	vector<float>			_prevP;
	vector<int>				_prevPos;
	uint					_frame;
public:
	//BVH				*_bvh;
	BVHHost				*_bvhHost;
	ObjectParams		h_objParams;
	vector< Obstacle*>	_obstacles;
public:
	vector<double>		_positions0;
	vector<double>		_masses;
public:
	int					_cloth_id;
public:
	VectorXd			_pHs;
	double				_Etotal;
public:
	ClothDynamics_PBD(void);
	ClothDynamics_PBD(Mesh *mesh, int type);
	~ClothDynamics_PBD(void);
public:
	inline void p(int i, Vector3d pos);
	inline void v(int i, Vector3d vel);
	inline void v_add(int i, Vector3d vel);
	inline double m(int i) { return _M.coeff(i * 3, i * 3); }
	inline Vector3d p(int i) { return Vector3d(_nodes[i * 3], _nodes[i * 3 + 1], _nodes[i * 3 + 2]); }
	inline Vector3d v(int i) { return Vector3d(_velocities[i * 3], _velocities[i * 3 + 1], _velocities[i * 3 + 2]); }
	inline Vec3 fv(int i, int j) { return Vec3(_nodes[_faces[i] * 3], _nodes[_faces[i] * 3 + j], _nodes[_faces[i] * 3 + j]); }
public:
	void clickVertex(Vec3 point);
	void moveVertex(Vec3 point);
	void resetCickVertex(void);
public:
	void	init(void);
	void	initMass(void);
	void	initConstraint(void);
	void	initBVH(void);
public:
	void	updateMass(void);
	void	reset(void);
	void	update(void);
	void	simulation(void);
	void	applyDamping(void);
	void	fixedPoints(void);
	void	computeNormal(void);
	void	computeExternalForce(void);
	void	applyAerodynamicForce(void);
public:
	void	collisions(void);
	void	addObstacle(Obstacle *obj);
	bool	obstacleCollisions(const ObjectParams& d_clothParams, double delta, double friction);
	bool	obstacleCollisionsHost(
		const vector<int>& faces,
		const vector<double>& vertices,
		vector<double>& velocities,
		const vector<double>& masses,
		double thickness, double friction);
public:
	void	draw(void);
	void	drawWire(void);
	void	drawSurface(void);
	void	drawPlane(double y);
public:
	void	initSound(void);
	void	calcSoundEffects(void);
	void	calcSoundEffects2(void);
	void	calcSoundEffects2apply(void);
public:
	void	computeCrumpling(void);
};

#endif