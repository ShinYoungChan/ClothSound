#include "CollisionSolverHost.h"

vector<Vec3> CollisionSolverHost::lineTest;

bool CollisionSolverHost::ResolveBoundaryCollision(
	const vector<double>& vertices,
	vector<double>& velocities,
	const vector<double>& masses,
	const Vec3& minBoundary, const Vec3& maxBoundary,
	double thickness, double friction, double dt,
	int cloth_id)
{
	bool result = false;
	for (int i = 0; i < vertices.size() / 3; i++) {
		bool apply = false;
		if (masses[i] <= 0.0)
			continue;
		Vec3 p(vertices[i * 3 + 0], vertices[i * 3 + 1], vertices[i * 3 + 2]);
		Vec3 vel(velocities[i * 3 + 0], velocities[i * 3 + 1], velocities[i * 3 + 2]);
		for (int j = 0; j < 3; j++) {
			Vec3 N(0.0, 0.0, 0.0);
			N[j] = 1.0;
			double dist = p[j] - minBoundary[j] - thickness;
			double newDist = dist + vel[j] * dt;
			if (newDist < 0.0) {
				SoundManager::_fric_vs[cloth_id] += Length(vel);
				//SoundManager::_fric_vs[cloth_id] += Length(relVelT);
				SoundManager::_fric_num[cloth_id]++;

				double A_newVelN = vel[j] - newDist / dt;
				Vec3 relVelT = vel - N * vel[j];
				Vec3 newRelVelT = relVelT *
					max(0.0, 1.0 - friction * (A_newVelN - vel[j]) / Length(relVelT));
				vel = (N * A_newVelN + newRelVelT);
				apply = true;
			}
			N[j] = -1.0;
			dist = maxBoundary[j] - p[j] - thickness;
			newDist = dist - vel[j] * dt;
			if (newDist < 0.0) {
				SoundManager::_fric_vs[cloth_id] += Length(vel);
				//SoundManager::_fric_vs[cloth_id] += Length(relVelT);
				SoundManager::_fric_num[cloth_id]++;

				double A_newVelN = -vel[j] - newDist / dt;
				Vec3 relVelT = vel + N * vel[j];
				Vec3 newRelVelT = relVelT *
					max(0.0, 1.0 - friction * (A_newVelN + vel[j]) / Length(relVelT));
				vel = (N * A_newVelN + newRelVelT);
				apply = true;
			}
		}
		if (apply) {
			velocities[i * 3 + 0] = vel.x;
			velocities[i * 3 + 1] = vel.y;
			velocities[i * 3 + 2] = vel.z;
			result = true;
		}
	}
	return result;
}

inline void resolveImpulse(
	int i0,
	vector<ResponseParam>& responseParams,
	const vector<double>& masses,
	Vec3 imp, double penetration)
{
	/*impulses[i0 * 3 + 0] += imp.x;
	impulses[i0 * 3 + 1] += imp.y;
	impulses[i0 * 3 + 2] += imp.z;*/
	if (masses[i0] > 0.0) {
		if (responseParams[i0].pene > penetration) {
			responseParams[i0].pene = penetration;
			responseParams[i0].setImpulse(imp);
		}
	}
	/*if (impulses[i0 * 3 + 0] * impulses[i0 * 3 + 0] + impulses[i0 * 3 + 1] * impulses[i0 * 3 + 1] + impulses[i0 * 3 + 2] * impulses[i0 * 3 + 2] < LengthSquared(imp)) {
		penetrations[i0] = penetration;
		impulses[i0 * 3 + 0] = imp.x;
		impulses[i0 * 3 + 1] = imp.y;
		impulses[i0 * 3 + 2] = imp.z;
	}*/
	//imp /= (double)(edges.index[i0 + 1] - edges.index[i0]);
	//for (int i = edges.index[i0]; i < edges.index[i0 + 1]; i++) {
	//	int ind = edges.array[i];
	//	if (ind == i1 || ind == i2)
	//		continue;
	//	if (masses[ind] > 0.0) {
	//		//contact_count[ind] += 1;
	//		Vec3 _imp = imp / masses[ind];
	//		impulses[ind * 3 + 0] += _imp.x;
	//		impulses[ind * 3 + 1] += _imp.y;
	//		impulses[ind * 3 + 2] += _imp.z;
	//	}
	//}
}
bool CollisionSolverHost::isObsContactTV(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double delta, double* t, double* w0, double* w1)
{
	double _w0, _w1;
	if (DetectionTV_CCD(p0, p1, p2, p3, q0, q1, q2, q3, COL_TV, t, &_w0, &_w1)) {
		if (w0) *w0 = _w0;
		if (w1) *w1 = _w1;
#ifdef CCD_PRINT_TRI_TV
		if (Debugging)
			printf("tri0 %f %f %f\n", _w0, _w1, *t);
#endif
		return true;
	}
	if (isContactTV_Proximity(q0, q1, q2, q3, delta, &_w0, &_w1)) {
		if (w0) *w0 = _w0;
		if (w1) *w1 = _w1;
		*t = 1.0;
#ifdef CCD_PRINT_TRI_TV
		if (Debugging)
			printf("tri0 %f %f %f\n", _w0, _w1, *t);
#endif
		return true;
	}
	return false;
}
bool CollisionSolverHost::isObsContactEE(
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& q0, const Vec3& q1, const Vec3& q2, const Vec3& q3,
	double delta, double* t, double* w0, double* w1)
{
	double _w0, _w1;
	if (DetectionEE_CCD(p0, p1, p2, p3, q0, q1, q2, q3, COL_EE, t, &_w0, &_w1)) {
		if (w0) *w0 = _w0;
		if (w1) *w1 = _w1;
#ifdef CCD_PRINT_EDGE_EE
		if (Debugging)
			printf("edge0 %f %f %f\n", _w0, _w1, *t);
#endif
		return true;
	}

	if (isContactEE_Proximity(q0, q1, q2, q3, delta, &_w0, &_w1)) {
		if (w0) *w0 = _w0;
		if (w1) *w1 = _w1;
		*t = 1.0;
#ifdef CCD_PRINT_TRI_TV
		if (Debugging)
			printf("edge0 %f %f %f\n", _w0, _w1, *t);
#endif
		return true;
	}
	return false;
}

bool CollisionSolverHost::resolveObsContactTV(
	uint i0, uint i1, uint i2, uint i3,
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3,
	const vector<double>& masses,
	const vector<double>& obs_masses,
	vector<ResponseParam>& responseParams,
	double delta, double friction, double dt,
	bool sampling, int cloth_id)
{
	bool isProximity = false;
	double stability = 0.5;
	double t, w0, w1;
	Vec3 q0 = p0 + v0 * dt;
	Vec3 q1 = p1 + v1 * dt;
	Vec3 q2 = p2 + v2 * dt;
	Vec3 q3 = p3 + v3 * dt;
	if (!isObsContactTV(p0, p1, p2, p3, q0, q1, q2, q3, delta, &t, &w0, &w1))
		return false;
	isProximity = t < 0.0;
	if (isProximity)
		t = 1.0;
	double w2 = 1.0 - w0 - w1;

#ifndef SDF_NORMAL
	double ht = t * dt * HALF_TIME;
	Vec3 q0m = p0 + v0 * ht;
	Vec3 q1m = p1 + v1 * ht;
	Vec3 q2m = p2 + v2 * ht;
	Vec3 q3m = p3 + v3 * ht;
	Vec3 norm = Normalize(q0m * w0 + q1m * w1 + q2m * w2 - q3m);
#else
	Vec3 norm = computeNormV(i3, obs_faces, obs_vertices, obs_velocities, obs_nbFaces, t * dt);
#endif

	Vec3 vc = w0 * v0 + w1 * v1 + w2 * v2;
	double dist = Dot(p0 * w0 + p1 * w1 + p2 * w2 - p3, norm) - delta;
	double newDist = dist + Dot(vc - v3, norm) * dt;
	if (newDist >= 0.0)
		return true;

	double vcn = Dot(vc, norm);
	double v3n = Dot(v3, norm);
	double n_vcn = vcn - newDist / dt;
	Vec3 vct = vc - vcn * norm;
	Vec3 v3t = v3 - v3n * norm;
	Vec3 relVT = vct - v3t;
	Vec3 n_relVT(0.0, 0.0, 0.0);
	double l_relVT = Length(relVT);
	if (l_relVT != 0.0) {
		n_relVT = max(1.0 - friction * (-newDist / dt) / l_relVT, 0.0) * relVT;
		if (sampling) {
			SoundManager::_fric_vs[cloth_id] += Length(v3 - vc);
			//SoundManager::_fric_vs[cloth_id] += Length(relVT);
			SoundManager::_fric_num[cloth_id]++;
		}
	}

	Vec3 n_vct = v3t + n_relVT;
	Vec3 imp = -norm * newDist / dt;// +n_vct - vct;
	imp *= stability;
	resolveImpulse(i0, responseParams, masses, imp, newDist);
	resolveImpulse(i1, responseParams, masses, imp, newDist);
	resolveImpulse(i2, responseParams, masses, imp, newDist);
	return true;
}
bool CollisionSolverHost::resolveObsContactVT(
	uint i0, uint i1, uint i2, uint i3,
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3,
	const vector<double>& masses,
	const vector<double>& obs_masses,
	vector<ResponseParam>& responseParams,
	double delta, double friction, double dt,
	bool sampling, int cloth_id)
{
	bool isProximity = false;
	double stability = 0.5;
	double t, w0, w1;
	Vec3 q0 = p0 + v0 * dt;
	Vec3 q1 = p1 + v1 * dt;
	Vec3 q2 = p2 + v2 * dt;
	Vec3 q3 = p3 + v3 * dt;
	if (!isObsContactTV(p0, p1, p2, p3, q0, q1, q2, q3, delta, &t, &w0, &w1))
		return false;
	isProximity = t < 0.0;
	if (isProximity)
		t = 1.0;
	double w2 = 1.0 - w0 - w1;

	double ht = t * dt * HALF_TIME;
	Vec3 q0m = p0 + v0 * ht;
	Vec3 q1m = p1 + v1 * ht;
	Vec3 q2m = p2 + v2 * ht;
	Vec3 q3m = p3 + v3 * ht;
	Vec3 norm = Normalize(q3m - q0m * w0 - q1m * w1 - q2m * w2);

	Vec3 vc = w0 * v0 + w1 * v1 + w2 * v2;
	double dist = Dot(p3 - p0 * w0 - p1 * w1 - p2 * w2, norm) - delta;
	double newDist = dist + Dot(v3 - vc, norm) * dt;
	if (newDist >= 0.0)
		return true;

	double v3n = Dot(v3, norm);
	double vcn = Dot(vc, norm);
	double n_v3n = v3n - newDist / dt;
	Vec3 v3t = v3 - v3n * norm;
	Vec3 vct = vc - vcn * norm;
	Vec3 relVT = v3t - vct;
	Vec3 n_relVT(0.0, 0.0, 0.0);
	double l_relVT = Length(relVT);
	if (l_relVT != 0.0) {
		n_relVT = max(1.0 - friction * (-newDist / dt) / l_relVT, 0.0) * relVT;
		if (sampling) {
			SoundManager::_fric_vs[cloth_id] += Length(v3 - vc);
			//SoundManager::_fric_vs[cloth_id] += Length(relVT);
			SoundManager::_fric_num[cloth_id]++;
		}
	}
	Vec3 n_v3t = vct + n_relVT;
	Vec3 imp = -norm * newDist / dt;// +n_v3t - v3t;
	imp *= stability;
	resolveImpulse(i3, responseParams, masses, imp, newDist);
	return true;
}
bool CollisionSolverHost::resolveObsContactEE(
	uint i0, uint i1, uint i2, uint i3,
	const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
	const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3,
	const vector<double>& masses,
	const vector<double>& obs_masses,
	vector<ResponseParam>& responseParams,
	double delta, double friction, double dt,
	bool sampling, int cloth_id)
{
	bool isProximity = false;
	double stability = 0.5;
	double t, w0, w1;
	Vec3 q0 = p0 + v0 * dt;
	Vec3 q1 = p1 + v1 * dt;
	Vec3 q2 = p2 + v2 * dt;
	Vec3 q3 = p3 + v3 * dt;
	if (!isObsContactEE(p0, p1, p2, p3, q0, q1, q2, q3, delta, &t, &w0, &w1))
		return false;;
	isProximity = t < 0.0;
	if (isProximity)
		t = 1.0;

#ifndef SDF_NORMAL
	double ht = t * dt * HALF_TIME;
	Vec3 q0m = p0 + v0 * ht;
	Vec3 q1m = p1 + v1 * ht;
	Vec3 q2m = p2 + v2 * ht;
	Vec3 q3m = p3 + v3 * ht;
	Vec3 norm = Normalize(q0m + (q1m - q0m) * w0 - q2m - (q3m - q2m) * w1);
#else
	Vec3 norm = computeNormE(i2, i3, obs_faces, obs_vertices, obs_velocities, obs_nbFaces, t * dt);
#endif

	Vec3 v01 = v0 + (v1 - v0) * w0;
	Vec3 v23 = v2 + (v3 - v2) * w1;
	double dist = Dot(p0 + (p1 - p0) * w0 - p2 - (p3 - p2) * w1, norm) - delta;
	double newDist = dist + Dot(v01 - v23, norm) * dt;
	if (newDist >= 0.0)
		return true;

	double v01n = Dot(v01, norm);
	double v23n = Dot(v23, norm);
	Vec3 v01t = v01 - v01n * norm;
	Vec3 v23t = v23 - v23n * norm;
	Vec3 relVT = v01t - v23t;
	Vec3 n_relVT(0.0, 0.0, 0.0);
	double l_relVT = Length(relVT);
	if (l_relVT != 0.0) {
		n_relVT = max(1.0 - friction * (-newDist / dt) / l_relVT, 0.0) * relVT;
		if (sampling) {
			SoundManager::_fric_vs[cloth_id] += Length(v01 - v23);
			//SoundManager::_fric_vs[cloth_id] += Length(relVT);
			SoundManager::_fric_num[cloth_id]++;
		}
	}

	Vec3 n_v01t = v23t + n_relVT;
	Vec3 imp = -norm * newDist / dt;// +n_v01t - v01t;
	imp *= stability;
	resolveImpulse(i0, responseParams, masses, imp, newDist);
	resolveImpulse(i1, responseParams, masses, imp, newDist);
	return true;
}

void CollisionSolverHost::resolveObstacleCollision(
	uint& num,
	const vector<int>& faces,
	const vector<double>& vertices,
	const vector<double>& velocities,
	const vector<double>& masses,
	const vector<int>& obs_faces,
	const vector<double>& obs_vertices,
	const vector<double>& obs_velocities,
	const vector<double>& obs_masses,
	BVHNodeHost* nodeL, BVHNodeHost* nodeR,
	vector<ResponseParam>& responseParams,
	double delta, double friction, double dt,
	bool sampling, int cloth_id)
{
	if (!nodeL->intersect(nodeR)) { return; }
	auto isLeafL = (nodeL->_childs[0] == nullptr);
	auto isLeafR = (nodeR->_childs[0] == nullptr);
	double t;
	double pt;
	if (!isLeafL && !isLeafR) {
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL->_childs[0], nodeR->_childs[0], responseParams, delta, friction, dt,
			sampling, cloth_id);
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL->_childs[1], nodeR->_childs[0], responseParams, delta, friction, dt,
			sampling, cloth_id);
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL->_childs[0], nodeR->_childs[1], responseParams, delta, friction, dt,
			sampling, cloth_id);
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL->_childs[1], nodeR->_childs[1], responseParams, delta, friction, dt,
			sampling, cloth_id);
	}
	else if (isLeafL && !isLeafR) {
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL, nodeR->_childs[0], responseParams, delta, friction, dt,
			sampling, cloth_id);
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL, nodeR->_childs[1], responseParams, delta, friction, dt,
			sampling, cloth_id);
	}
	else if (!isLeafL && isLeafR) {
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL->_childs[0], nodeR, responseParams, delta, friction, dt,
			sampling, cloth_id);
		resolveObstacleCollision(num,
			faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			nodeL->_childs[1], nodeR, responseParams, delta, friction, dt,
			sampling, cloth_id);
	}
	else {

		Vec3 pi[3], vi[3], pj[3], vj[3];
		uint ino[3], jno[3];

		for (auto i : nodeL->_faces) {
			ino[0] = faces[i * 3 + 0];
			ino[1] = faces[i * 3 + 1];
			ino[2] = faces[i * 3 + 2];
			pi[0] = Vec3(vertices[ino[0] * 3 + 0], vertices[ino[0] * 3 + 1], vertices[ino[0] * 3 + 2]);
			pi[1] = Vec3(vertices[ino[1] * 3 + 0], vertices[ino[1] * 3 + 1], vertices[ino[1] * 3 + 2]);
			pi[2] = Vec3(vertices[ino[2] * 3 + 0], vertices[ino[2] * 3 + 1], vertices[ino[2] * 3 + 2]);
			vi[0] = Vec3(velocities[ino[0] * 3 + 0], velocities[ino[0] * 3 + 1], velocities[ino[0] * 3 + 2]);
			vi[1] = Vec3(velocities[ino[1] * 3 + 0], velocities[ino[1] * 3 + 1], velocities[ino[1] * 3 + 2]);
			vi[2] = Vec3(velocities[ino[2] * 3 + 0], velocities[ino[2] * 3 + 1], velocities[ino[2] * 3 + 2]);
			for (auto j : nodeR->_faces) {
				jno[0] = obs_faces[j * 3 + 0];
				jno[1] = obs_faces[j * 3 + 1];
				jno[2] = obs_faces[j * 3 + 2];
				pj[0] = Vec3(obs_vertices[jno[0] * 3 + 0], obs_vertices[jno[0] * 3 + 1], obs_vertices[jno[0] * 3 + 2]);
				pj[1] = Vec3(obs_vertices[jno[1] * 3 + 0], obs_vertices[jno[1] * 3 + 1], obs_vertices[jno[1] * 3 + 2]);
				pj[2] = Vec3(obs_vertices[jno[2] * 3 + 0], obs_vertices[jno[2] * 3 + 1], obs_vertices[jno[2] * 3 + 2]);
				vj[0] = Vec3(obs_velocities[jno[0] * 3 + 0], obs_velocities[jno[0] * 3 + 1], obs_velocities[jno[0] * 3 + 2]);
				vj[1] = Vec3(obs_velocities[jno[1] * 3 + 0], obs_velocities[jno[1] * 3 + 1], obs_velocities[jno[1] * 3 + 2]);
				vj[2] = Vec3(obs_velocities[jno[2] * 3 + 0], obs_velocities[jno[2] * 3 + 1], obs_velocities[jno[2] * 3 + 2]);
				for (uint k = 0; k < 3; k++) {
					if (nodeR->RTriVertex(k))
						if (resolveObsContactTV(
							ino[0], ino[1], ino[2], jno[k],
							pi[0], pi[1], pi[2], pj[k],
							vi[0], vi[1], vi[2], vj[k],
							masses, obs_masses, responseParams,
							delta, friction, dt, sampling, cloth_id))
							num++;
					if (nodeL->RTriVertex(k))
						if (resolveObsContactVT(
							jno[0], jno[1], jno[2], ino[k],
							pj[0], pj[1], pj[2], pi[k],
							vj[0], vj[1], vj[2], vi[k],
							masses, obs_masses, responseParams,
							delta, friction, dt, sampling, cloth_id))
							num++;

					if (nodeL->RTriVertex(k)) {
						uint i1 = (k + 1) % 3;
						for (uint l = 0; l < 3; l++) {
							uint j1 = (l + 1) % 3;
							if (nodeL->RTriVertex(l))
								if (resolveObsContactEE(
									ino[k], ino[i1], jno[l], jno[j1],
									pi[k], pi[i1], pj[l], pj[j1],
									vi[k], vi[i1], vj[l], vj[j1],
									masses, obs_masses, responseParams,
									delta, friction, dt, sampling, cloth_id))
									num++;
						}
					}
				}
			}
		}
	}
}

bool CollisionSolverHost::ResolveObstacleCollision(
	const vector<int>& faces,
	const vector<double>& vertices,
	vector<double>& velocities,
	const vector<double>& masses,
	BVHHost* bvh,
	const vector<int>& obs_faces,
	const vector<double>& obs_vertices,
	vector<double>& obs_velocities,
	const vector<double>& obs_masses,
	BVHHost* obs_bvh,
	double thickness, double friction, double dt,
	int cloth_id)
{
	bool result = false;
	vector<ResponseParam> responseParams;
	uint num;
	printf("--------------------\n");
	for (int itr = 0; itr < 15; itr++) {
		bvh->refit(faces, vertices, velocities, thickness, dt, true);
		obs_bvh->refit(obs_faces, obs_vertices, obs_velocities, thickness, dt, true);
		responseParams.resize(velocities.size());
		num = 0;
		resolveObstacleCollision(
			num, faces, vertices, velocities, masses,
			obs_faces, obs_vertices, obs_velocities, obs_masses,
			bvh->_root, obs_bvh->_root, responseParams, thickness, friction, dt, 
			true, cloth_id);
		if (!num)
			break;
		printf("Obstacle %d, %d\n", itr, num);
		for (int i = 0; i < vertices.size() / 3; i++) {
			if (responseParams[i].pene > 0.0)
				continue;
			velocities[i * 3 + 0] += responseParams[i].impulse[0];
			velocities[i * 3 + 1] += responseParams[i].impulse[1];
			velocities[i * 3 + 2] += responseParams[i].impulse[2];
		}
		result = true;
		responseParams.clear();
	}
	return result;
}