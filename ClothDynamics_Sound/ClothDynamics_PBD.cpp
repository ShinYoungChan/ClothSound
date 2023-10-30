#include "ClothDynamics_PBD.h"

ClothDynamics_PBD::ClothDynamics_PBD(void)
{
}

ClothDynamics_PBD::~ClothDynamics_PBD(void)
{
}

ClothDynamics_PBD::ClothDynamics_PBD(Mesh *mesh, int type)
{
	_mesh = mesh;
	_type = type;
	_constraint = new Constraint();
	init();
	_clickID = -1;
	_frame = 0;
	initSound();
}

void ClothDynamics_PBD::p(int i, Vector3d pos)
{
	for (int j = 0; j < 3; j++) {
		_nodes[i * 3 + j] = pos[j];
	}
}

void ClothDynamics_PBD::v_add(int i, Vector3d vel)
{
	for (int j = 0; j < 3; j++) {
		_velocities[i * 3 + j] += vel[j];
	}
}

void ClothDynamics_PBD::v(int i, Vector3d vel)
{
	for (int j = 0; j < 3; j++) {
		_velocities[i * 3 + j] = vel[j];
	}
}

void ClothDynamics_PBD::addObstacle(Obstacle *obj)
{
	_obstacles.push_back(obj);
}

void ClothDynamics_PBD::clickVertex(Vec3 point) {
	if (_clickID > -1)
		return;
	double mindist = 0.05;
	int minIndex = -1;
	for (int i = 0; i < _numNodes; i++) {
		Vec3 p(_nodes[i * 3 + 0], _nodes[i * 3 + 1], _nodes[i * 3 + 2]);

		//double dist = (p - point).Length();
		double dist = (p.x - point.x) * (p.x - point.x) + (p.y - point.y) * (p.y - point.y);
		if (dist < mindist) {
			mindist = dist;
			minIndex = i;
		}
	}
	_clickID = minIndex;
	if (_clickID > -1) {
		_fixedPoints.push_back(_clickID);
		updateMass();
		_nodes[_clickID * 3 + 0] = point.x;
		_nodes[_clickID * 3 + 1] = point.y;
		//_nodes[_clickID * 3 + 2] = 0.15;
		_velocities[_clickID * 3 + 0] = 0.0;
		_velocities[_clickID * 3 + 1] = 0.0;
		_velocities[_clickID * 3 + 2] = 0.0;
	}
}
void ClothDynamics_PBD::moveVertex(Vec3 point) {
	if (_clickID > -1) {
		_nodes[_clickID * 3 + 0] = point.x;
		_nodes[_clickID * 3 + 1] = point.y;
		//_nodes[_clickID * 3 + 2] = 0.05;
	}
}
void ClothDynamics_PBD::resetCickVertex(void) {
	if (_clickID > -1) {
		if (_fixedPoints.back() == _clickID) {
			_clickID = -1;
			_fixedPoints.erase(_fixedPoints.begin() + _fixedPoints0, _fixedPoints.end());
			updateMass();
		}
	}
}

void ClothDynamics_PBD::init(void)
{
	_maxIter = 30;
	//_maxIter = 150;
	_dt = 0.01;
	//_dt = SOUND_DELTATIME;

	_mass = 1.0;

	_numNodes = _mesh->_vertices.size();
	int size = _numNodes * 3;
	//double offset = 0.000001;
	double offset = 0.1;

	_nodes.resize(size);
	_velocities.resize(size);
	_positions0.resize(size);
	_pHs.resize(_numNodes);

	_fixedPoints.clear();
	for (auto v : _mesh->_vertices) {
		auto index = v->_index;
		for (int i = 0; i < 3; i++) {
			_nodes[3 * index + i] = v->_pos[i];
			_velocities[3 * index + i] = 0.0;
		}
		_pHs[index] = 0.0;

		// hanging cloth1
		/*if (v->x() >= _mesh->_maxBoundary.x - offset && v->y() >= _mesh->_maxBoundary.y - offset) {
			_fixedPoints.push_back(index);
		}*/
		/*if ((v->x() <= _mesh->_minBoundary.x + offset ||
			v->x() >= _mesh->_maxBoundary.x - offset) && v->y() >= _mesh->_maxBoundary.y - offset) {
			_fixedPoints.push_back(index);
		}*/
		if (v->y() >= _mesh->_maxBoundary.y - offset) {
			_fixedPoints.push_back(index);
		}

		/*if (v->x() <= _mesh->_minBoundary.x + offset && v->y() >= _mesh->_maxBoundary.y - offset) {
			_fixedPoints.push_back(index);
		}
		if (v->x() >= _mesh->_maxBoundary.x - offset && v->y() >= _mesh->_maxBoundary.y - offset) {
			_fixedPoints.push_back(index);
		}
		if (v->x() <= _mesh->_minBoundary.x + offset && v->y() <= _mesh->_minBoundary.y + offset) {
			_fixedPoints.push_back(index);
		}
		if (v->x() >= _mesh->_maxBoundary.x - offset && v->y() <= _mesh->_minBoundary.y + offset) {
			_fixedPoints.push_back(index);
		}*/

		/*if (v->x() <= _mesh->_minBoundary.x + offset) {
			_fixedPoints.push_back(index);
		}
		if (v->x() >= _mesh->_maxBoundary.x - offset) {
			_fixedPoints.push_back(index);
		}
		if (v->y() <= _mesh->_minBoundary.y + offset) {
			_fixedPoints.push_back(index);
		}
		if (v->y() >= _mesh->_maxBoundary.y - offset) {
			_fixedPoints.push_back(index);
		}*/
	}
	_fixedPoints0 = _fixedPoints.size();
	_nnormals = MatrixXd::Zero(3, _mesh->_vertices.size());
	for (int i = 0; i < (int)_mesh->_vertices.size(); i++) {
		auto v = _mesh->_vertices[i];
		for (int j = 0; j < 3; j++)
			_nnormals(j, i) = v->_normal[j];
	}

	int index = 0;
	_normals = MatrixXd::Zero(3, _mesh->_faces.size());
	_faces.resize(_mesh->_faces.size() * 3);

	for (int i = 0; i < (int)_mesh->_faces.size(); i++) {
		auto f = _mesh->_faces[i];
		for (int j = 0; j < 3; j++) {
			_faces[index + j] = f->v(j)->_index;
			_normals(j, i) = f->_normal[j];
		}
		index += 3;
	}

	initMass();
	initConstraint();
	initBVH();
}

void ClothDynamics_PBD::initMass(void)
{
	_masses.clear();
	_masses.resize(_numNodes, _mass);
	_M.resize(_numNodes * 3, _numNodes * 3);
	_invMasses.resize(_numNodes * 3, _numNodes * 3);
	vector<Eigen::Triplet<double>> m_coeff;
	double fixedPointMass = 0.0;

	for (int i = 0; i < _numNodes * 3; i++) {
		if (find(_fixedPoints.begin(), _fixedPoints.end(), i / 3) != _fixedPoints.end()) {
			m_coeff.push_back(Eigen::Triplet<double>(i, i, fixedPointMass));
		}
		else {
			m_coeff.push_back(Eigen::Triplet<double>(i, i, 1.0));
		}
	}
	_M.setFromTriplets(m_coeff.begin(), m_coeff.end());
	_M *= _mass;
	_M.makeCompressed();

	_invMasses.setFromTriplets(m_coeff.begin(), m_coeff.end());
	_invMasses *= 1.0 / _mass;
	_invMasses.makeCompressed();
}

void ClothDynamics_PBD::initConstraint(void)
{
	_constraint->init(_mesh, _faces);
	printf("num. constraints : %d\n", _constraint->_springs.size());
}

void ClothDynamics_PBD::initBVH(void) {
	ParamManager::initObjectParamHost(h_objParams, _faces.rows() / 3, _nodes.rows() / 3, true);
	_positions0.resize(_nodes.rows());
	_masses.resize(_nodes.rows() / 3);

	vector< set<int> > edges;
	vector< vector<int> > nbfaces;
	int nodeSize = _nodes.rows() / 3;
	edges.resize(nodeSize);
	nbfaces.resize(nodeSize);

	NbFaceBuffer h_nbFaces;
	h_nbFaces.index = (int*)malloc((nodeSize + 1) * sizeof(int));
	h_nbFaces.index[0] = 0;

	for (int i = 0; i < h_objParams.fnum; i++) {
		for (int j = 0; j < 3; j++) {
			int ino = _faces[i * 3 + j];
			h_objParams.faces[i * 3 + j] = ino;
			edges[ino].insert(_faces[i * 3 + ((j + 1) % 3)]);
			edges[ino].insert(_faces[i * 3 + ((j + 2) % 3)]);
			nbfaces[ino].push_back(i);
			h_nbFaces.index[ino + 1] = nbfaces[ino].size();
		}
	}
	_edges.resize(nodeSize);
	for (int i = 1; i < nodeSize; i++)
		h_nbFaces.index[i + 1] += h_nbFaces.index[i];
	h_nbFaces.array = (int*)malloc(h_nbFaces.index[nodeSize] * sizeof(int));
	for (int i = 0; i < nodeSize; i++) {
		int ino = 0;
		for (set<int>::iterator itr = edges[i].begin(); itr != edges[i].end(); itr++)
			_edges[i].push_back(*itr);

		for (int j = 0; j < nbfaces[i].size(); j++)
			h_nbFaces.array[h_nbFaces.index[i] + j] = nbfaces[i][j];

		for (int j = 0; j < 3; j++)
			h_objParams.vertices[i * 3 + j] = _nodes[i * 3 + j];
		h_objParams.masses[i] = m(i);
	}

	//_bvh = new BVH(h_objParams, h_nbFaces);
	_bvhHost = new BVHHost(h_objParams, h_nbFaces);
	free(h_nbFaces.index);
	free(h_nbFaces.array);
}

void ClothDynamics_PBD::updateMass(void)
{
	_M.resize(_numNodes * 3, _numNodes * 3);
	_invMasses.resize(_numNodes * 3, _numNodes * 3);
	vector<Eigen::Triplet<double>> m_coeff;
	double fixedPointMass = 0.0;

	for (int i = 0; i < _numNodes * 3; i++) {
		if (find(_fixedPoints.begin(), _fixedPoints.end(), i / 3) != _fixedPoints.end())
			m_coeff.push_back(Eigen::Triplet<double>(i, i, fixedPointMass));
		else
			m_coeff.push_back(Eigen::Triplet<double>(i, i, 1.0));
	}

	_M.setFromTriplets(m_coeff.begin(), m_coeff.end());
	for (int i = 0; i < _numNodes; i++)
		for (int j = 0; j < 3; j++)
			_M.coeffRef(i * 3 + j, i * 3 + j) *= _masses[i];
	_M.makeCompressed();

	_invMasses.setFromTriplets(m_coeff.begin(), m_coeff.end());
	for (int i = 0; i < _numNodes; i++) {
		if (_masses[i] == 0) continue;
		for (int j = 0; j < 3; j++)
			_invMasses.coeffRef(i * 3 + j, i * 3 + j) /= _masses[i];
	}
	_invMasses.makeCompressed();
}

void ClothDynamics_PBD::simulation(void)
{
	computeExternalForce();
	update();
	//applyDamping();
	collisions();

	//calcSoundEffects();
	//calcSoundEffects2();
	computeNormal();
	computeCrumpling();

	_frame++;
}

void ClothDynamics_PBD::applyAerodynamicForce(void)
{
	int selected_cond = 4;
	
	constexpr tuple<double, double, double, const char*> conditions[] = {
		{0.000, 0.000, 0.0, "without-aerodynamics"},
		{0.060, 0.030, 0.0, "with-aerodynamics"},
		{0.080, 0.030, 8.0, "wind"},
		{0.080, 0.000, 8.0, "wind-high-drag"},
		{0.080, 0.080, 4.0, "wind-high-lift"},
	};
		
	double user_weight = 11.0;
	const auto &cond = conditions[selected_cond];
	auto drag_coeff = get<0>(cond) * user_weight;
	auto lift_coeff = get<1>(cond) * user_weight;
	auto wind_velocity = get<2>(cond) * user_weight;
	auto condition_name = get<3>(cond);
	
	constexpr double rho = 1.225; // Taken from Wikipedia: https://en.wikipedia.org/wiki/Density_of_air
	
	int numFaces = _mesh->_faces.size();
	for (int i = 0; i < numFaces; i++) {
		auto f = _mesh->_faces[i];
		Vector3d x_0 = p(f->_vertices[0]->_index);
		Vector3d x_1 = p(f->_vertices[1]->_index);
		Vector3d x_2 = p(f->_vertices[2]->_index);

		Vector3d v_0 = v(f->_vertices[0]->_index);
		Vector3d v_1 = v(f->_vertices[1]->_index);
		Vector3d v_2 = v(f->_vertices[2]->_index);
	
		double m_0 = m(f->_vertices[0]->_index);
		double m_1 = m(f->_vertices[1]->_index);
		double m_2 = m(f->_vertices[2]->_index);

		const double mass_sum = m_0 + m_1 + m_2;
		if (!mass_sum)
			continue;
		// Calculate the weighted average of the particle velocities
		const Vector3d v_triangle = (m_0 * v_0 + m_1 * v_1 + m_2 * v_2) / mass_sum;

		//// Calculate the relative velocity of the triangle
		Vector3d global_velocity(0.0, 0.0, wind_velocity);
		const Vector3d v_rel = v_triangle - global_velocity;
		const double v_rel_squared = v_rel.squaredNorm();
		const Vector3d cross = (x_1 - x_0).cross(x_2 - x_0);
		const double area = 0.5 * cross.norm();
		const Vector3d n_either_side = cross.normalized();
		//const Vector3d n = (n_either_side.dot(v_rel) > 0.0) ? n_either_side : -n_either_side;
		Vector3d n;
		if (n_either_side.dot(v_rel) > 0.0)
			n = n_either_side;
		else
			n = -n_either_side;

		const double coeff = 0.5 * rho * area;

		// Note: This wind force model was proposed by [Wilson+14]
		const Vector3d force = -coeff * ((drag_coeff - lift_coeff) * v_rel.dot(n) * v_rel + lift_coeff * v_rel_squared * n);
		double masses[3] = {m_0, m_1, m_2};

		for (int j = 0; j < 3; j++) {
			int v_id = f->_vertices[j]->_index;
			for (int k = 0; k < 3; k++) {
				_force[v_id * 3 + k] += (masses[j] / mass_sum) * force[k];
			}
			//printf("%f, %f, %f\n", _force[v_id * 3 + 0], _force[v_id * 3 + 1], _force[v_id * 3 + 2]);
		}
	}
}

void ClothDynamics_PBD::applyDamping(void)
{
	double damping_coeff = 0.1;
	double airDampingCoeff = 0.9999;

	Vector3d avgPos(0, 0, 0);
	Vector3d avgVel(0, 0, 0);
	double denominator = 0.0;

	for (int i = 0; i < _numNodes; i++) {
		double mass = _M.coeff(i * 3, i * 3);
		avgPos += mass * p(i);
		avgVel += mass * v(i);
		denominator += mass;
	}

	avgPos /= denominator;
	avgVel /= denominator;

	Vector3d angular_momentum(0.0, 0.0, 0.0);
	Matrix3d rot_mat, inertia;
	rot_mat.setZero();
	inertia.setZero();

	for (int i = 0; i < _numNodes; i++) {
		double mass = _M.coeff(i * 3, i * 3);
		Vector3d r = p(i) - avgPos;
		angular_momentum += r.cross(v(i));

		rot_mat.coeffRef(0, 1) = r[2];
		rot_mat.coeffRef(0, 2) = -r[1];
		rot_mat.coeffRef(1, 0) = -r[2];
		rot_mat.coeffRef(1, 2) = r[0];
		rot_mat.coeffRef(2, 0) = r[1];
		rot_mat.coeffRef(2, 1) = -r[0];

		inertia += rot_mat * rot_mat.transpose() * mass;
	}

	Vector3d angular_vel = inertia.inverse() * angular_momentum;
	Vector3d delta_v(0.0, 0.0, 0.0);

	for (int i = 0; i < _numNodes; i++) {
		Vector3d r = p(i) - avgPos;
		delta_v = avgVel + angular_vel.cross(r) - v(i);
		v_add(i, damping_coeff * delta_v);
		for (int j = 0; j < 3; j++) {
			_velocities[i * 3 + j] *= airDampingCoeff;
		}
	}

	fixedPoints();
}

void ClothDynamics_PBD::fixedPoints(void)
{
	for (auto f : _fixedPoints) {
		v(f, Vector3d(0, 0, 0));
	}
}

void ClothDynamics_PBD::collisions(void)
{
	for (auto obs : _obstacles)
		obs->update(_dt);

	bool isApplied = false;

	bool appliedImpulse = false;
	double contactClearance = 0.1;
	double thickness = 0.01;
	double clothFriction = 1.0;
	double boundaryFriction = 0.5;
	double obstacleFriction = 1.0;

	vector<int> faces;
	vector<double> velocities;
	vector<double> masses;

	int size = _faces.size() / 3;
	for (int i = 0; i < size; i++)
		for (int j = 0; j < 3; j++)
			faces.push_back(_faces[i * 3 + j]);

	for (int i = 0; i < _numNodes; i++) {
		for (int j = 0; j < 3; j++)
			velocities.push_back(_velocities[i * 3 + j]);
		masses.push_back(m(i));
	}

	Vec3 minBoundary(-1.5, -1.75, -1.5);
	Vec3 maxBoundary(1.5, 1.5, 1.5);

	//appliedImpulse |= CollisionSolverHost::ResolveBoundaryCollision(_positions0, velocities, masses, minBoundary, maxBoundary, thickness, 1.0 - boundaryFriction, _dt, _cloth_id);
	appliedImpulse |= obstacleCollisionsHost(faces, _positions0, velocities, masses, thickness, 1.0 - obstacleFriction);
	appliedImpulse |= CollisionSolverHost::ResolveSelfCollision(faces, _positions0, velocities, _edges, masses, _bvhHost, contactClearance, 1.0 - clothFriction, _dt, _cloth_id);

	for (int i = 0; i < _numNodes; i++) {
		for (int j = 0; j < 3; j++) {
			_velocities[i * 3 + j] = velocities[i * 3 + j];
			_nodes[i * 3 + j] = _positions0[i * 3 + j] + velocities[i * 3 + j] * _dt;
		}
	}
}

bool ClothDynamics_PBD::obstacleCollisions(const ObjectParams& d_clothParams, double delta, double friction) {
	bool result = false;
	/*for (auto obs : _obstacles) {
		ObjectParams d_obsParams;
		ParamManager::initObjectParamDevice(d_obsParams, obs->h_objParams, true);
		result |= CollisionSolver::ResolveObstacleCollision(d_clothParams, d_obsParams, _bvh, obs->_bvh, delta, friction, _dt);
		ParamManager::destroyObjectParamDevice(d_obsParams, true);
	}*/
	return result;
}
bool ClothDynamics_PBD::obstacleCollisionsHost(
	const vector<int>& faces,
	const vector<double>& vertices,
	vector<double>& velocities,
	const vector<double>& masses,
	double thickness, double friction)
{
	bool result = false;
	for (auto obs : _obstacles) {
		vector<int> obs_faces;
		vector<double> obs_vertices, obs_velocities;

		for (int i = 0; i < obs->_nodes.rows(); i++) {
			auto vel = obs->v(i);
			for (int j = 0; j < 3; j++) {
				obs_velocities.push_back(vel[j]);
			}
		}

		for (int i = 0; i < obs->_faces.rows(); i++)
			for (int j = 0; j < 3; j++)
				obs_faces.push_back(obs->fv(i, j));

		result |= CollisionSolverHost::ResolveObstacleCollision(
			faces, vertices, velocities, masses, _bvhHost,
			obs_faces, obs->_pos0, obs_velocities, obs->_masses, obs->_bvhHost,
			thickness, friction, _dt, _cloth_id);
	}
	return result;
}
void ClothDynamics_PBD::initSound(void) {
	_receiver.Set(0.0, 0.0, 13.5);
	//_receiver.Set(5.5, -0.78, 1.0);
	float beta = 1.f;
	_w = ceilf(3.f / ((float)MAX_FREQUENCY * SOUND_DELTATIME)) * beta;
	beta = 1.f / beta;
	float mx = 0.f;
	{
		//_gw = SOUND_FRAME;
		/*float gw = _w + _w;
		_gkernel.resize(_gw + _gw + 1);
		float sigma = (float)_gw / sqrtf(8.f * logf(2)) * 0.5;
		float c = 2.f * sigma * sigma;
		float sc = 1.f / (sqrtf(2 * M_PI) * sigma);
		for (int i = -_gw; i <= _gw; i++)
			_gkernel[i + _gw] = exp(-i * i / c) * sc;*/
		_gw = _w + _w;
		_gkernel.resize(_gw + _gw + 1);
		float sigma = (float)_gw / sqrtf(8.f * logf(2));
		float c = 2.f * sigma * sigma;
		for (int i = -_gw; i <= _gw; i++) {
			_gkernel[i + _gw] = exp(-i * i / c);
			mx += _gkernel[i + _gw];
		}
		for (int i = 0; i <= _gw + _gw; i++)
			_gkernel[i] /= mx;
	}
	mx = -1.f;
	_skernel.resize(_w + _w + 1);
	for (int i = -_w; i <= _w; i++) {
		float sinc = 
			sinf(2.f * M_PI * i * SOUND_DELTATIME * (float)MAX_FREQUENCY * beta) /
			(M_PI * i * SOUND_DELTATIME + FLT_EPSILON * beta);
		float win = 0.5 + cosf(M_PI * (float)i / (float)_w) * 0.5;
		_skernel[-i + _w] = sinc * win;
		if (mx < fabsf(_skernel[-i + _w]))
			mx = fabsf(_skernel[-i + _w]);
	}
	for (int i = 0; i <= _w + _w; i++)
		_skernel[i] /= mx;
}
void ClothDynamics_PBD::calcSoundEffects(void)
{
	float z = 415.0;
	float c = 343.0;
	/*vector<float> gs(w + w + 1);
	vector<float> ps(w + w + 1);
	vector<float> ss(w + w + 1);
	for (int i = 0; i < _faces.size() / 3; i++) {
		int ino0 = _faces[i * 3 + 0];
		int ino1 = _faces[i * 3 + 1];
		int ino2 = _faces[i * 3 + 2];
		Vec3 p0(_nodes[ino0 * 3 + 0], _nodes[ino0 * 3 + 1], _nodes[ino0 * 3 + 2]);
		Vec3 p1(_nodes[ino1 * 3 + 0], _nodes[ino1 * 3 + 1], _nodes[ino1 * 3 + 2]);
		Vec3 p2(_nodes[ino2 * 3 + 0], _nodes[ino2 * 3 + 1], _nodes[ino2 * 3 + 2]);
		Vec3 v0(_velocities[ino0 * 3 + 0], _velocities[ino0 * 3 + 1], _velocities[ino0 * 3 + 2]);
		Vec3 v1(_velocities[ino1 * 3 + 0], _velocities[ino1 * 3 + 1], _velocities[ino1 * 3 + 2]);
		Vec3 v2(_velocities[ino2 * 3 + 0], _velocities[ino2 * 3 + 1], _velocities[ino2 * 3 + 2]);
		Vec3 norm = Cross(p1 - p0, p2 - p0);
		float a = norm.Length();
		norm *= 1.0 / a;
		a *= 0.5;

		float p = z / 3.0 * Dot(v0 + v1 + v2, norm);
		for (int j = -w; j <= w; j++) {
			float sinc = sinf(2 * M_PI * j * sdt * fmax) /
			(M_PI * j * sdt + FLT_EPSILON);
			float win = 0.5 + cosf(M_PI * (float)j / (float)w) * 0.5;
			float K = sinc * win;
			gs[j + w] = K * p;
		}
		ps[0] = gs[0];
		for (int j = 1; j < 2 * w + 1; j++) {
			ps[j] = 0.9 * ps[j - 1] + gs[j] - gs[j - 1];
		}
		Vec3 xr = receiver - (1.0 / 3.0) * (p0 + p1 + p2);
		float xrlen = LengthSquared(xr);
		//float sc = a * Dot(xr, norm) / xrlen;
		float sc = fabs(a * Dot(xr, norm) / xrlen);
		for (int j = 0; j < 2 * w + 1; j++) {
			ss[j] = ps[j] * sc;
		}
		xrlen = sqrt(xrlen);
		int d = round(xrlen / c * 44100.0);
		int pos = _frame * 1470 + d;
		if (_buffer.size() < pos + 1470 + w)
			_buffer.resize(pos + 1470 + w + 1, 0.f);
		for (int i = 0; i < 1470; i++) {
			for (int j = -w; j <= w; j++)
				_buffer[max(0, pos + i + j)] += ss[j + w];
		}
	}*/
	vector<float> ss(_w + _w + 1);
	for (int i = 0; i < _faces.size() / 3; i++) {
		int ino0 = _faces[i * 3 + 0];
		int ino1 = _faces[i * 3 + 1];
		int ino2 = _faces[i * 3 + 2];
		Vec3 p0(_nodes[ino0 * 3 + 0], _nodes[ino0 * 3 + 1], _nodes[ino0 * 3 + 2]);
		Vec3 p1(_nodes[ino1 * 3 + 0], _nodes[ino1 * 3 + 1], _nodes[ino1 * 3 + 2]);
		Vec3 p2(_nodes[ino2 * 3 + 0], _nodes[ino2 * 3 + 1], _nodes[ino2 * 3 + 2]);
		Vec3 v0(_velocities[ino0 * 3 + 0], _velocities[ino0 * 3 + 1], _velocities[ino0 * 3 + 2]);
		Vec3 v1(_velocities[ino1 * 3 + 0], _velocities[ino1 * 3 + 1], _velocities[ino1 * 3 + 2]);
		Vec3 v2(_velocities[ino2 * 3 + 0], _velocities[ino2 * 3 + 1], _velocities[ino2 * 3 + 2]);
		Vec3 norm = Cross(p1 - p0, p2 - p0);
		float a = norm.Length();
		norm *= 1.0 / a;
		a *= 0.5;
		
		float p = z / 3.0 * Dot(v0 + v1 + v2, norm);
		float prevg;
		for (int j = -_w; j <= _w; j++) {
			float gi = p * _skernel[j + _w];
			if (j == -_w)
				ss[0] = gi;
			else
				ss[j + _w] = 0.9 * ss[j + _w - 1] + (gi - prevg);
			prevg = gi;
		}

		Vec3 xr = _receiver - (1.0 / 3.0) * (p0 + p1 + p2);
		float xrlen = LengthSquared(xr) + FLT_EPSILON;
		float sc = a * Dot(xr, norm) / xrlen;
		
		for (int j = 0; j < _w + _w + 1; j++)
			ss[j] *= sc;

		xrlen = sqrtf(xrlen);
		int d = round(xrlen / c * 44100.0);
		int pos = _frame * SOUND_FRAME + d;
		if (_buffer.size() < pos + _w + 1)
			_buffer.resize(pos + _w + 1, 0.0);

		/*for (int itr = 0; itr < 1; itr++) {
			float gw = _w + _w;
			vector<float> smooths(_w + _w + 1);
			for (int j = 0; j < _w + _w + 1; j++) {
				float smooth = 0.f;
				for (int k = -gw; k <= gw; k++)
					if (j + k >= 0 && j + k < _w + _w + 1)
						smooth += ss[j + k] * _gkernel[k + gw];
				smooths[j] = smooth;
			}
			ss = smooths;
		}*/

		for (int j = -_w; j <= _w; j++)
			if (pos + j >= 0) {
				_buffer[pos + j] += ss[j + _w];
				//_buffer[pos + j] = max(_buffer[pos + j], ss[j + _w]);
			}

		/*for (int itr = 0; itr < 1; itr++) {
			float gw = _w + _w;
			vector<float> smooths(_w + _w + 1);
			for (int j = -_w; j <= _w; j++) {
				if (pos + j >= 0 && pos + j < _buffer.size()) {
					float smooth = 0.f;
					for (int k = -gw; k <= gw; k++)
						if (pos + j + k >= 0 && pos + j + k < _buffer.size())
							smooth += _buffer[pos + j + k] * _gkernel[k + gw];
					smooths[_w + j] = smooth;
				}
			}
			for (int j = -_w; j <= _w; j++)
				if (pos + j >= 0 && pos + j < _buffer.size())
					_buffer[pos + j] = smooths[_w + j];
		}*/
		/*for (int itr = 0; itr < 1; itr++) {
			float gw = _w + _w;
			vector<float> smooths(gw + gw + 1);
			for (int j = -gw; j <= gw; j++) {
				if (pos + j >= 0 && pos + j < _buffer.size()) {
					float smooth = 0.f;
					for (int k = -gw; k <= gw; k++)
						if (pos + j + k >= 0 && pos + j + k < _buffer.size())
							smooth += _buffer[pos + j + k] * _gkernel[k + gw];
					smooths[gw + j] = smooth;
				}
			}
			for (int j = -gw; j <= gw; j++) {
				if (pos + j >= 0 && pos + j < _buffer.size()) {
					_buffer[pos + j] = smooths[gw + j];
				}
			}
		}*/
#ifdef SOUND_SUBDIV
		if (_frame > 0) {
			int divf = floorf((float)SOUND_FRAME / (float)SUBDIV_FRAME);
			float invdiv = 1.0f / (float)SUBDIV_FRAME;
			for (int alp = 1; alp < SUBDIV_FRAME; alp++) {
				int ppos = 
					(int)roundf((float)_prevPos[i] * (1.f - (float)alp * invdiv)) +
					(int)roundf((float)pos * (float)alp * invdiv);
				for (int j = -_w; j <= _w; j++) {
					if (pos + j >= 0) {
						_buffer[ppos + j] +=
							(1.f - (float)alp * invdiv) * _prevS[i][j + _w] +
							(float)alp * invdiv * ss[j + _w];
						/*_buffer[ppos + j] = max(_buffer[ppos + j],
							(1.f - (float)alp * invdiv) * _prevS[i][j + _w] +
							(float)alp * invdiv * ss[j + _w]);*/
					}
				}
			}
		}
		if (_prevS.size() < i + 1)
			_prevS.resize(i + 1);
		_prevS[i] = ss;
		if (_prevPos.size() < i + 1)
			_prevPos.resize(i + 1, 0);
		_prevPos[i] = pos;
#endif
	}
	/*printf("-------------\n");
	for (auto i : _buffer) {
		if (fabs(i) > 1.0e-10)
			printf("%f\n", i);
	}*/
	/*{
		vector<int> faces;
		vector<double> vertices;
		vector<double> velocities;

		int size = _faces.size() / 3;
		for (int i = 0; i < size; i++)
			for (int j = 0; j < 3; j++)
				faces.push_back(_faces[i * 3 + j]);

		for (int i = 0; i < _numNodes; i++) {
			for (int j = 0; j < 3; j++) {
				vertices.push_back(_nodes[i * 3 + j]);
				velocities.push_back(_velocities[i * 3 + j]);
			}
		}
		_bvhHost->refit(faces, vertices, velocities, 0.0, _dt, false);
	}
	for (int i = 0; i < _faces.size() / 3; i++) {
		int ino0 = _faces[i * 3 + 0];
		int ino1 = _faces[i * 3 + 1];
		int ino2 = _faces[i * 3 + 2];
		Vec3 p0(_nodes[ino0 * 3 + 0], _nodes[ino0 * 3 + 1], _nodes[ino0 * 3 + 2]);
		Vec3 p1(_nodes[ino1 * 3 + 0], _nodes[ino1 * 3 + 1], _nodes[ino1 * 3 + 2]);
		Vec3 p2(_nodes[ino2 * 3 + 0], _nodes[ino2 * 3 + 1], _nodes[ino2 * 3 + 2]);
		Vec3 v0(_velocities[ino0 * 3 + 0], _velocities[ino0 * 3 + 1], _velocities[ino0 * 3 + 2]);
		Vec3 v1(_velocities[ino1 * 3 + 0], _velocities[ino1 * 3 + 1], _velocities[ino1 * 3 + 2]);
		Vec3 v2(_velocities[ino2 * 3 + 0], _velocities[ino2 * 3 + 1], _velocities[ino2 * 3 + 2]);
		Vec3 norm = Cross(p1 - p0, p2 - p0);
		float a = norm.Length();
		norm *= 1.0 / a;
		a *= 0.5;
		Vec3 xr = receiver - (1.0 / 3.0) * (p0 + p1 + p2);
		float xrlen = Length(xr);
		xr *= 1.0 / xrlen;
		if (_bvhHost->rayTracing(receiver, -xr) != i)
			continue;

		float p = z / 3.0 * Dot(v0 + v1 + v2, norm);
		for (int j = -w; j <= w; j++) {
			float sinc = sinf(2 * M_PI * j * sdt * fmax) / (M_PI * j * sdt + FLT_EPSILON);
			float win = 0.5 + cosf(M_PI * (float)j / (float)w) * 0.5;
			float K = sinc * win;
			gs[j + w] = K * p;
		}
		ps[0] = gs[0];
		for (int j = 1; j < 2 * w + 1; j++) {
			ps[j] = 0.9 * ps[j - 1] + gs[j] - gs[j - 1];
		}
		float sc = fabs(a * Dot(xr, norm) / xrlen);
		for (int j = 0; j < 2 * w + 1; j++) {
			ss[j] = ps[j] * sc;
		}
		int d = round(xrlen / c * 44100.0);
		int pos = _frame * 1470 + d;
		if (_buffer.size() < pos + w + 1)
			_buffer.resize(pos + w + 1, 0.f);

		for (int j = -w; j <= w; j++)
			_buffer[max(0, pos + j)] += ss[j + w];
	}*/
}
void ClothDynamics_PBD::calcSoundEffects2(void)
{
	float z = 415.0;
	float c = 343.0;
	for (int i = 0; i < _faces.size() / 3; i++) {
		int ino0 = _faces[i * 3 + 0];
		int ino1 = _faces[i * 3 + 1];
		int ino2 = _faces[i * 3 + 2];
		Vec3 p0(_nodes[ino0 * 3 + 0], _nodes[ino0 * 3 + 1], _nodes[ino0 * 3 + 2]);
		Vec3 p1(_nodes[ino1 * 3 + 0], _nodes[ino1 * 3 + 1], _nodes[ino1 * 3 + 2]);
		Vec3 p2(_nodes[ino2 * 3 + 0], _nodes[ino2 * 3 + 1], _nodes[ino2 * 3 + 2]);
		Vec3 v0(_velocities[ino0 * 3 + 0], _velocities[ino0 * 3 + 1], _velocities[ino0 * 3 + 2]);
		Vec3 v1(_velocities[ino1 * 3 + 0], _velocities[ino1 * 3 + 1], _velocities[ino1 * 3 + 2]);
		Vec3 v2(_velocities[ino2 * 3 + 0], _velocities[ino2 * 3 + 1], _velocities[ino2 * 3 + 2]);
		Vec3 norm = Cross(p1 - p0, p2 - p0);
		float a = norm.Length();
		norm *= 1.0 / a;
		a *= 0.5;

		float p = z / 3.0 * Dot(v0 + v1 + v2, norm);
		Vec3 xr = _receiver - (1.0 / 3.0) * (p0 + p1 + p2);
		float xrlen = LengthSquared(xr) + FLT_EPSILON;
		float sc = a * Dot(xr, norm) / xrlen;
		p *= sc;

		xrlen = sqrtf(xrlen);
		int d = round(xrlen / c * 44100.0);
		int pos = _frame * SOUND_FRAME + d;
		if (_buffer.size() < pos)
			_buffer.resize(pos, 0.0);

		/*for (int itr = 0; itr < 1; itr++) {
			float gw = _w + _w;
			vector<float> smooths(_w + _w + 1);
			for (int j = 0; j < _w + _w + 1; j++) {
				float smooth = 0.f;
				for (int k = -gw; k <= gw; k++)
					if (j + k >= 0 && j + k < _w + _w + 1)
						smooth += ss[j + k] * _gkernel[k + gw];
				smooths[j] = smooth;
			}
			ss = smooths;
		}*/

		_buffer[pos] += p;

		/*for (int itr = 0; itr < 1; itr++) {
			float gw = _w + _w;
			vector<float> smooths(_w + _w + 1);
			for (int j = -_w; j <= _w; j++) {
				if (pos + j >= 0 && pos + j < _buffer.size()) {
					float smooth = 0.f;
					for (int k = -gw; k <= gw; k++)
						if (pos + j + k >= 0 && pos + j + k < _buffer.size())
							smooth += _buffer[pos + j + k] * _gkernel[k + gw];
					smooths[_w + j] = smooth;
				}
			}
			for (int j = -_w; j <= _w; j++)
				if (pos + j >= 0 && pos + j < _buffer.size())
					_buffer[pos + j] = smooths[_w + j];
		}*/
		/*for (int itr = 0; itr < 1; itr++) {
			float gw = _w + _w;
			vector<float> smooths(gw + gw + 1);
			for (int j = -gw; j <= gw; j++) {
				if (pos + j >= 0 && pos + j < _buffer.size()) {
					float smooth = 0.f;
					for (int k = -gw; k <= gw; k++)
						if (pos + j + k >= 0 && pos + j + k < _buffer.size())
							smooth += _buffer[pos + j + k] * _gkernel[k + gw];
					smooths[gw + j] = smooth;
				}
			}
			for (int j = -gw; j <= gw; j++) {
				if (pos + j >= 0 && pos + j < _buffer.size()) {
					_buffer[pos + j] = smooths[gw + j];
				}
			}
		}*/
#ifdef SOUND_SUBDIV
		if (_frame > 0) {
			int divf = floorf((float)SOUND_FRAME / (float)SUBDIV_FRAME);
			float invdiv = 1.0f / (float)SUBDIV_FRAME;
			for (int alp = 1; alp < SUBDIV_FRAME; alp++) {
				int ppos =
					(int)roundf((float)_prevPos[i] * (1.f - (float)alp * invdiv)) +
					(int)roundf((float)pos * (float)alp * invdiv);
				if (pos >= 0) {
					_buffer[ppos] +=
						(1.f - (float)alp * invdiv) * _prevP[i] +
						(float)alp * invdiv * p;
				}
			}
		}
		if (_prevP.size() < i + 1)
			_prevP.resize(i + 1);
		_prevP[i] = p;
		if (_prevPos.size() < i + 1)
			_prevPos.resize(i + 1, 0);
		_prevPos[i] = pos;
#endif
	}
}
void ClothDynamics_PBD::calcSoundEffects2apply(void) {
	/*for (int itr = 0; itr < 1; itr++) {
		vector<float> smooths(_buffer.size());
		for (int i = 0; i < (int)_buffer.size(); i++) {
			float smooth = 0.f;
			for (int j = -_gw; j <= _gw; j++)
				if (i + j >= 0 && i + j < _buffer.size())
					smooth += _buffer[i + j] * _gkernel[j + _gw];
			smooths[i] = smooth;
		}
		_buffer = smooths;
	}*/

	vector<float> gs(_buffer.size());
	for (int i = 0; i < (int)_buffer.size(); i++) {
		float data = 0.0f;
		for (int j = -_w; j <= _w; j++) {
			int pos = i + j;
			if (pos >= 0 && pos < _buffer.size()) {
				data += _buffer[pos] * _skernel[j + _w];// *_gkernel[j + _gw - _w];
			}
		}
		gs[i] = data;
	}
	_buffer[0] = gs[0];
	for (int i = 1; i < (int)_buffer.size(); i++)
		_buffer[i] = _buffer[i - 1] * 0.9 + gs[i] - gs[i - 1];
}

void ClothDynamics_PBD::computeCrumpling(void) {
	_Etotal = 0.0;
	for (int i = 0; i < _numNodes; i++) {
		double pH = _pHs[i];
		int size = _edges[i].size();
		if (!size)
			continue;
		double H = 0.0;
		auto edges = _edges[i];
		int pi = edges[0];
		Vec3 pv(_nodes[pi * 3 + 0], _nodes[pi * 3 + 1], _nodes[pi * 3 + 2]);
		Vec3 pnorm(_nnormals(0, pi), _nnormals(1, pi), _nnormals(2, pi));
		while (edges.size()) {
			bool end = false;
			for (int ie = 0; ie < edges.size(); ie++) {
				int e0 = edges[ie];
				if (e0 == pi) continue;
				for (auto e1 : _edges[pi]) {
					if (e0 == e1) {
						Vec3 nv(_nodes[e0 * 3 + 0], _nodes[e0 * 3 + 1], _nodes[e0 * 3 + 2]);
						Vec3 nnorm(_nnormals(0, pi), _nnormals(1, e0), _nnormals(2, e0));

						H += Dot(nnorm - pnorm, nv - pv) / LengthSquared(nv - pv);

						pi = e0;
						pv = nv;
						pnorm = nnorm;
						end = true;
						break;
					}
				}
				if (end) {
					edges.erase(edges.begin() + ie);
					break;
				}
			}
			if (!end) {
				size--;
				break;
			}
		}
		H /= (double)size;
		_pHs[i] = H;
		if (H * pH < 0) {
			SoundManager::_crump_num[_cloth_id]++;
			//H -= pH;
			H += pH;
			_Etotal += H * H;
		}
	}
	SoundManager::_crump_Es[_cloth_id] = _Etotal;
	printf("%f\n", _Etotal);
}

void ClothDynamics_PBD::update(void)
{
	VectorXd &x = _nodes;
	VectorXd &v = _velocities;

	v = v + _dt * _invMasses * _force;
	VectorXd p = x;
	p = x + _dt * v;

	// project
	if (_type == 0) {
		// PBD
		_constraint->project(p, _invMasses, _maxIter);
	}
	else if (_type == 1) {
		// XPBD
		_constraint->projectXPBD(p, _invMasses, _dt, _maxIter);
	}

	//_constraint->tearing(_faces, p, x, _masses, _edges);

	//update positions0
	for (int i = _positions0.size(); i < x.size(); i++)
		_positions0.push_back(x[i]);

	//update the v and x
	v = (p - x) / _dt;
	x = p;

	_numNodes = x.size() / 3;
	_normals.conservativeResize(3, _faces.size() / 3);
	_nnormals.conservativeResize(3, _nodes.size() / 3);
	updateMass();
}

void ClothDynamics_PBD::computeNormal(void)
{
	int numFaces = _mesh->_faces.size();
	/*for (int i = 0; i < numFaces; i++) {
		auto f = _mesh->_faces[i];
		Vector3f a(_nodes[f->_vertices[0]->_index * 3], _nodes[f->_vertices[0]->_index * 3 + 1], _nodes[f->_vertices[0]->_index * 3 + 2]);
		Vector3f b(_nodes[f->_vertices[1]->_index * 3], _nodes[f->_vertices[1]->_index * 3 + 1], _nodes[f->_vertices[1]->_index * 3 + 2]);
		Vector3f c(_nodes[f->_vertices[2]->_index * 3], _nodes[f->_vertices[2]->_index * 3 + 1], _nodes[f->_vertices[2]->_index * 3 + 2]);
		Vector3f a2b = b - a;
		Vector3f a2c = c - a;
		Vector3f normal = a2b.cross(a2c);
		normal.normalize();
		_normals(0, i) = normal.x();
		_normals(1, i) = normal.y();
		_normals(2, i) = normal.z();
	}*/
	for (int i = 0; i < _numNodes; i++) {
		_nnormals(0, i) = 0.0;
		_nnormals(1, i) = 0.0;
		_nnormals(2, i) = 0.0;
	}

	for (int i = 0; i < numFaces; i++) {
		int i0 = _faces[i * 3 + 0];
		int i1 = _faces[i * 3 + 1];
		int i2 = _faces[i * 3 + 2];
		Vector3f a(_nodes[i0 * 3], _nodes[i0 * 3 + 1], _nodes[i0 * 3 + 2]);
		Vector3f b(_nodes[i1 * 3], _nodes[i1 * 3 + 1], _nodes[i1 * 3 + 2]);
		Vector3f c(_nodes[i2 * 3], _nodes[i2 * 3 + 1], _nodes[i2 * 3 + 2]);
		Vector3f a2b = b - a;
		Vector3f a2c = c - a;
		Vector3f normal = a2b.cross(a2c);
		normal.normalize();
		_normals(0, i) = normal.x();
		_normals(1, i) = normal.y();
		_normals(2, i) = normal.z();

		/*Vector3f n = normal * AngleBetweenVectors(a2b, a2c);
		_nnormals(0, i0) += n.x();
		_nnormals(1, i0) += n.y();
		_nnormals(2, i0) += n.z();
		n = normal * AngleBetweenVectors(a2b, b - c);
		_nnormals(0, i1) += n.x();
		_nnormals(1, i1) += n.y();
		_nnormals(2, i1) += n.z();
		n = normal * AngleBetweenVectors(a2c, c - b);
		_nnormals(0, i2) += n.x();
		_nnormals(1, i2) += n.y();
		_nnormals(2, i2) += n.z();*/
		_nnormals(0, i0) += normal.x();
		_nnormals(1, i0) += normal.y();
		_nnormals(2, i0) += normal.z();
		_nnormals(0, i1) += normal.x();
		_nnormals(1, i1) += normal.y();
		_nnormals(2, i1) += normal.z();
		_nnormals(0, i2) += normal.x();
		_nnormals(1, i2) += normal.y();
		_nnormals(2, i2) += normal.z();
	}

	for (int i = 0; i < _numNodes; i++) {
		auto x = _nnormals(0, i);
		auto y = _nnormals(1, i);
		auto z = _nnormals(2, i);
		auto len = sqrtf(x * x + y * y + z * z);
		if (len > 0) {
			len = 1.f / len;
			_nnormals(0, i) *= len;
			_nnormals(1, i) *= len;
			_nnormals(2, i) *= len;
		}
	}
}

void ClothDynamics_PBD::reset(void)
{
	/*_numNodes = _mesh->_vertices.size();
	_nodes.resize(_numNodes * 3);
	_force.setZero();
	_velocities.setZero();

	for (auto v : _mesh->_vertices) {
		auto index = v->_index;
		for (int i = 0; i < 3; i++) {
			_nodes[3 * index + i] = v->_pos[i];
		}
	}
	for (int i = _constraints.size() - 1; i >= 0; i--) {
		auto c = _constraints[i];
		_constraints.pop_back();
		delete c;
	}
	initConstraint();*/
	_constraint->clear();
	for (int i = 0; i < _edges.size(); i++)
		_edges[i].clear();
	_edges.clear();
	init();
}

void ClothDynamics_PBD::computeExternalForce(void)
{
	int size = _numNodes * 3;
	VectorXd force(size);
	force.setZero();

	double radius = 0.3;
	double f = -15000;

	for (int i = 0; i < _numNodes; i++) {
		force[i * 3 + 1] += -9.8; // gravity
		/*if (_frame == 50) {
			if (_nodes[i * 3 + 0] > -radius && _nodes[i * 3 + 0] < radius &&
				_nodes[i * 3 + 1] > -radius && _nodes[i * 3 + 1] < radius) {
				force[i * 3 + 2] += f;
			}
		}*/
		for (int j = 0; j < 3; j++) {
			_positions0[i * 3 + j] = _nodes[i * 3 + j];
		}
	}

	_force = _M * force;

	//applyAerodynamicForce();
}

void ClothDynamics_PBD::draw(void)
{
	drawWire();
	drawSurface();
	//_bvhHost->draw();

	for (auto obs : _obstacles)
		obs->draw();

	for (int i = 0; i < _testPoints.size(); i++) {
		glPushMatrix();
		glTranslated(_testPoints[i].x, _testPoints[i].y, _testPoints[i].z);
		glutSolidSphere(0.01, 10, 10);
		glPopMatrix();
	}
	for (auto v : _constraint->_testPoint) {
		glPushMatrix();
		glTranslated(v.x, v.y, v.z);
		glutSolidSphere(0.01, 10, 10);
		glPopMatrix();
	}
	/*if (_clickID > -1) {
		glPushMatrix();
		glTranslated(_nodes[_clickID * 3 + 0], _nodes[_clickID * 3 + 1], _nodes[_clickID * 3 + 2]);
		glutSolidSphere(0.05, 10, 10);
		glPopMatrix();
	}*/
	//drawPlane(-1.75);
}

void ClothDynamics_PBD::drawPlane(double y)
{
	glPushMatrix();
	glBegin(GL_QUADS);
	glVertex3f(-2.0, y, -2.0);
	glVertex3f(2.0, y, -2.0);
	glVertex3f(2.0, y, 2.0);
	glVertex3f(-2.0, y, 2.0);
	glEnd();
	glPopMatrix();
}

void ClothDynamics_PBD::drawWire(void)
{
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glColor3d(0, 0, 0);
	for (int i = 0; i < _faces.size(); i += 3) {
		glm::vec3 a(_nodes[_faces[i] * 3], _nodes[_faces[i] * 3 + 1], _nodes[_faces[i] * 3 + 2]);
		glm::vec3 b(_nodes[_faces[i + 1] * 3], _nodes[_faces[i + 1] * 3 + 1], _nodes[_faces[i + 1] * 3 + 2]);
		glm::vec3 c(_nodes[_faces[i + 2] * 3], _nodes[_faces[i + 2] * 3 + 1], _nodes[_faces[i + 2] * 3 + 2]);
		glBegin(GL_LINE_LOOP);
		glVertex3f(a.x, a.y, a.z);
		glVertex3f(b.x, b.y, b.z);
		glVertex3f(c.x, c.y, c.z);
		glEnd();
	}
	glEnable(GL_LIGHTING);
	glShadeModel(GL_FLAT);
	glPopMatrix();
}

void ClothDynamics_PBD::drawSurface(void)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1); // two-sided lighting.
	float purple[] = { 0.0f, 0.44705882352941176470588235294118f, 0.66666666666666666666666666666667f, 1.0f };
	float yellow[] = { 0.6f, 0.6f, 0.0f, 1.0f };
	float white[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	float black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, purple);
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
	glMaterialf(GL_FRONT, GL_SHININESS, 64);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, yellow); // back material
	glMaterialfv(GL_BACK, GL_SPECULAR, black); // no specular highlights

	for (int i = 0; i < _faces.size(); i += 3) {
		glm::vec3 a(_nodes[_faces[i] * 3], _nodes[_faces[i] * 3 + 1], _nodes[_faces[i] * 3 + 2]);
		glm::vec3 b(_nodes[_faces[i + 1] * 3], _nodes[_faces[i + 1] * 3 + 1], _nodes[_faces[i + 1] * 3 + 2]);
		glm::vec3 c(_nodes[_faces[i + 2] * 3], _nodes[_faces[i + 2] * 3 + 1], _nodes[_faces[i + 2] * 3 + 2]);
		glBegin(GL_TRIANGLES);
		glNormal3f(_normals(0, i / 3), _normals(1, i / 3), _normals(2, i / 3));
		glVertex3f(a.x, a.y, a.z);
		glVertex3f(b.x, b.y, b.z);
		glVertex3f(c.x, c.y, c.z);
		glEnd();
	}
}