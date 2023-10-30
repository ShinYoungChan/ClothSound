#include "Constraint.h"

Constraint::Constraint()
{
}
Constraint::~Constraint()
{
}

void Constraint::init(Mesh* mesh, VectorXi& faces)
{
	_triangles.resize(mesh->_faces.size() * 3);
	for (auto e : mesh->_stretchEdges) {
		int e0 = e->v(0)->_index;
		int e1 = e->v(1)->_index;
		auto p0 = e->_nbFaces[0]->otherVertex(e);
		int f0 = e->_nbFaces[0]->_index;
		int v0 = p0->_index;
		int f1 = -1;
		int v1 = -1;
		double vlen;

		for (int i = 0; i < 3; i++) {
			int ino = f0 * 3 + i;
			if (faces[ino] != e0 && faces[ino] != e1) {
				_triangles[ino] = _springs.size();
				break;
			}
		}
		if (e->_nbFaces.size() == 1) {
			vlen = ((e->v(0)->_pos + e->v(1)->_pos) * 0.5 - p0->_pos).Length();
		}
		else {
			auto p1 = e->_nbFaces[1]->otherVertex(e);
			f1 = e->_nbFaces[1]->_index;
			v1 = p1->_index;
			for (int i = 0; i < 3; i++) {
				int ino = f1 * 3 + i;
				if (faces[ino] != e0 && faces[ino] != e1) {
					_triangles[ino] = _springs.size();
					break;
				}
			}
			vlen = (p1->_pos - p0->_pos).Length();
		}
		_springs.push_back(new Spring(0.9, e0, e1, v0, v1, f0, f1, e->length(), vlen, e->v(1)->_pos - e->v(0)->_pos));
	}
}
void Constraint::clear(void) {
	for (int i = _springs.size() - 1; i >= 0; i--) {
		auto spring = _springs[i];
		_springs.pop_back();
		delete spring;
	}
}

void Constraint::project(VectorXd& x, SparseMatrixXd& inv_masses, int iter)
{
	for (int itr = 0; itr < iter; itr++) {
		for (int i = 0; i < _springs.size(); i++) {
			auto spring = _springs[i];
			double k_prime = 1.0 - pow(1.0 - spring->_stiffness, 1.0 / (double)iter);

			Vec3 p0(x[spring->_ps[0] * 3], x[spring->_ps[0] * 3 + 1], x[spring->_ps[0] * 3 + 2]);
			Vec3 p1(x[spring->_ps[1] * 3], x[spring->_ps[1] * 3 + 1], x[spring->_ps[1] * 3 + 2]);

			double curr_length = (p0 - p1).Length();
			Vec3 curr_dir = (p0 - p1) / curr_length;
			Vec3 displacement = (curr_length - spring->_length) * curr_dir;

			double w0 = inv_masses.coeff(spring->_ps[0] * 3, spring->_ps[0] * 3);
			double w1 = inv_masses.coeff(spring->_ps[1] * 3, spring->_ps[1] * 3);

			p0 -= k_prime * displacement * w0 / (w0 + w1);
			p1 += k_prime * displacement * w1 / (w0 + w1);

			for (int i = 0; i < 3; i++) {
				x[spring->_ps[0] * 3 + i] = p0[i];
				x[spring->_ps[1] * 3 + i] = p1[i];
			}

#ifdef BENDING
			if (spring->_fs[1] > -1) {
				p0.Set(x[spring->_ps[2] * 3], x[spring->_ps[2] * 3 + 1], x[spring->_ps[2] * 3 + 2]);
				p1.Set(x[spring->_ps[3] * 3], x[spring->_ps[3] * 3 + 1], x[spring->_ps[3] * 3 + 2]);

				curr_length = (p0 - p1).Length();
				curr_dir = (p0 - p1) / curr_length;
				displacement = (curr_length - spring->_vlength) * curr_dir;

				w0 = inv_masses.coeff(spring->_ps[2] * 3, spring->_ps[2] * 3);
				w1 = inv_masses.coeff(spring->_ps[3] * 3, spring->_ps[3] * 3);

				p0 -= k_prime * displacement * w0 / (w0 + w1);
				p1 += k_prime * displacement * w1 / (w0 + w1);

				for (int i = 0; i < 3; i++) {
					x[spring->_ps[2] * 3 + i] = p0[i];
					x[spring->_ps[3] * 3 + i] = p1[i];
				}
			}
#endif
		}
	}
}
void Constraint::projectXPBD(VectorXd& x, SparseMatrixXd& inv_masses, double dt, int iter)
{
//	double material = 0.00000000000016;
//	//double material = 0.000001;
//	material /= dt * dt;
//	for (auto spring : _springs) {
//		spring->_lambda = 0.0;
//		spring->_lambda2 = 0.0;
//	}
//
//	for (int itr = 0; itr < iter; itr++) {
//		for (int i = 0; i < _springs.size(); i++) {
//			auto spring = _springs[i];
//			double w0 = inv_masses.coeff(spring->_ps[0] * 3, spring->_ps[0] * 3);
//			double w1 = inv_masses.coeff(spring->_ps[1] * 3, spring->_ps[1] * 3);
//			if (w0 + w1 <= 0)
//				continue;
//			Vec3 p0(x[spring->_ps[0] * 3], x[spring->_ps[0] * 3 + 1], x[spring->_ps[0] * 3 + 2]);
//			Vec3 p1(x[spring->_ps[1] * 3], x[spring->_ps[1] * 3 + 1], x[spring->_ps[1] * 3 + 2]);
//			Vec3 curr_dir = p1 - p0;
//			double curr_length = curr_dir.Length();
//			double constraint = curr_length - spring->_length;
//			double dt_lambda = (-constraint - material * spring->_lambda) / ((w0 + w1) + material);
//			Vec3 corr_vector = dt_lambda * curr_dir / (curr_length + FLT_EPSILON);
//			spring->_lambda += dt_lambda;
//
//			p0 -= w0 * corr_vector;
//			p1 += w1 * corr_vector;
//
//			for (int i = 0; i < 3; i++) {
//				x[spring->_ps[0] * 3 + i] = p0[i];
//				x[spring->_ps[1] * 3 + i] = p1[i];
//			}
//#ifdef BENDING
//			if (spring->_fs[1] > -1) {
//				w0 = inv_masses.coeff(spring->_ps[2] * 3, spring->_ps[2] * 3);
//				w1 = inv_masses.coeff(spring->_ps[3] * 3, spring->_ps[3] * 3);
//				p0.Set(x[spring->_ps[2] * 3], x[spring->_ps[2] * 3 + 1], x[spring->_ps[2] * 3 + 2]);
//				p1.Set(x[spring->_ps[3] * 3], x[spring->_ps[3] * 3 + 1], x[spring->_ps[3] * 3 + 2]);
//				curr_dir = p1 - p0;
//				curr_length = curr_dir.Length();
//				constraint = curr_length - spring->_vlength;
//				dt_lambda = (-constraint - material * spring->_lambda2) / ((w0 + w1) + material);
//				corr_vector = dt_lambda * curr_dir / (curr_length + FLT_EPSILON);
//				spring->_lambda2 += dt_lambda;
//
//				p0 -= w0 * corr_vector;
//				p1 += w1 * corr_vector;
//
//				for (int i = 0; i < 3; i++) {
//					x[spring->_ps[2] * 3 + i] = p0[i];
//					x[spring->_ps[3] * 3 + i] = p1[i];
//				}
//			}
//#endif
//		}
//	}

	for (int itr = 0; itr < iter; itr++) {
		for (int i = 0; i < _springs.size(); i++) {
			auto spring = _springs[i];
			double w0 = inv_masses.coeff(spring->_ps[0] * 3, spring->_ps[0] * 3);
			double w1 = inv_masses.coeff(spring->_ps[1] * 3, spring->_ps[1] * 3);
			if (w0 + w1 <= 0)
				continue;
			Vec3 p0(x[spring->_ps[0] * 3], x[spring->_ps[0] * 3 + 1], x[spring->_ps[0] * 3 + 2]);
			Vec3 p1(x[spring->_ps[1] * 3], x[spring->_ps[1] * 3 + 1], x[spring->_ps[1] * 3 + 2]);
			Vec3 curr_dir = p1 - p0;
			double curr_length = curr_dir.Length();
			double constraint = curr_length - spring->_length;
			double dt_lambda = (-constraint) / ((w0 + w1) + FLT_EPSILON);
			Vec3 corr_vector = dt_lambda * curr_dir / (curr_length + FLT_EPSILON);

			p0 -= w0 * corr_vector;
			p1 += w1 * corr_vector;

			for (int i = 0; i < 3; i++) {
				x[spring->_ps[0] * 3 + i] = p0[i];
				x[spring->_ps[1] * 3 + i] = p1[i];
			}
#ifdef BENDING
			if (spring->_fs[1] > -1) {
				w0 = inv_masses.coeff(spring->_ps[2] * 3, spring->_ps[2] * 3);
				w1 = inv_masses.coeff(spring->_ps[3] * 3, spring->_ps[3] * 3);
				p0.Set(x[spring->_ps[2] * 3], x[spring->_ps[2] * 3 + 1], x[spring->_ps[2] * 3 + 2]);
				p1.Set(x[spring->_ps[3] * 3], x[spring->_ps[3] * 3 + 1], x[spring->_ps[3] * 3 + 2]);
				curr_dir = p1 - p0;
				curr_length = curr_dir.Length();
				constraint = curr_length - spring->_vlength;
				dt_lambda = (-constraint) / ((w0 + w1) + FLT_EPSILON);
				corr_vector = dt_lambda * curr_dir / (curr_length + FLT_EPSILON);

				p0 -= w0 * corr_vector;
				p1 += w1 * corr_vector;

				for (int i = 0; i < 3; i++) {
					x[spring->_ps[2] * 3 + i] = p0[i];
					x[spring->_ps[3] * 3 + i] = p1[i];
				}
			}
#endif
		}
	}
}

void Constraint::addEdge(EdgeBufferHost& edges, int p0, int p1) {
	edges[p0].push_back(p1);
	edges[p1].push_back(p0);
}
void Constraint::delEdge(EdgeBufferHost& edges, int p0, int p1) {
	for (int e = 0; e < edges[p0].size(); e++) {
		if (edges[p0][e] == p1) {
			edges[p0].erase(edges[p0].begin() + e);
			break;
		}
		if (e == edges[p0].size() - 1)
			printf("Error Constarint::delEdge\n");
	}
	for (int e = 0; e < edges[p1].size(); e++) {
		if (edges[p1][e] == p0) {
			edges[p1].erase(edges[p1].begin() + e);
			break;
		}
		if (e == edges[p1].size() - 1)
			printf("Error Constarint::delEdge\n");
	}
}

int Constraint::addNode(VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges, const Vec3& pos, const Vec3& ppos) {
	int nodeSize = x.size() / 3;
	x.conservativeResize(nodeSize * 3 + 3);
	x[nodeSize * 3 + 0] = pos.x;
	x[nodeSize * 3 + 1] = pos.y;
	x[nodeSize * 3 + 2] = pos.z;

	px.conservativeResize(nodeSize * 3 + 3);
	px[nodeSize * 3 + 0] = ppos.x;
	px[nodeSize * 3 + 1] = ppos.y;
	px[nodeSize * 3 + 2] = ppos.z;

	edges.push_back(vector<int>(0));
	masses.push_back(1.0);
	return nodeSize;
}
void Constraint::addSpring(EdgeBufferHost& edges, int p0, int p1, int v0, int v1, int f0, int f1, double length, double vlength, Vec3 vec, double teartheta) {
	_springs.push_back(new Spring(0.9, p0, p1, v0, v1, f0, f1, length, vlength, vec, teartheta));
}
int Constraint::addFace(VectorXi& faces, int n0, int n1, int n2, int s0, int s1, int s2, int* inodes_ord, int* isprings_ord) {
	int faceSize = faces.size() / 3;
	faces.conservativeResize(faceSize * 3 + 3);
	faces[faceSize * 3 + inodes_ord[0]] = n0;
	faces[faceSize * 3 + inodes_ord[1]] = n1;
	faces[faceSize * 3 + inodes_ord[2]] = n2;
	_triangles.conservativeResize(faceSize * 3 + 3);
	_triangles[faceSize * 3 + isprings_ord[0]] = s0;
	_triangles[faceSize * 3 + isprings_ord[1]] = s1;
	_triangles[faceSize * 3 + isprings_ord[2]] = s2;
	return faceSize;
}
void Constraint::cleanup(
	VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
	Spring* spring, int pivot)
{
	Vec3 p(x[pivot * 3 + 0], x[pivot * 3 + 1], x[pivot * 3 + 2]);
	Vec3 pp(px[pivot * 3 + 0], px[pivot * 3 + 1], px[pivot * 3 + 2]);
	int if0 = spring->_fs[0];
	int if1 = spring->_fs[1];
	int edgeNum = edges[pivot].size() - 1;

	int connectedNum0 = abs(getAdjNum(spring, spring, if1, pivot));
	if (connectedNum0 == edgeNum)
		return;

	if (if1 < 0) {
		Vec3 dir(
			x[spring->_cs[0] * 3 + 0] - p.x,
			x[spring->_cs[0] * 3 + 1] - p.y,
			x[spring->_cs[0] * 3 + 2] - p.z
		);
		dir.SetNormalizedVector();
		dir *= 1.0e-3;
		int np = addNode(x, px, masses, edges, p + dir, pp + dir);
		double mass = masses[pivot] * 0.5;
		masses[pivot] = masses[np] = mass;
		adjustVertex(faces, edges, spring, -1, pivot, np);
	}
	else {
		Vec3 dir(
			x[spring->_cs[0] * 3 + 0] - x[spring->_cs[1] * 3 + 0],
			x[spring->_cs[0] * 3 + 1] - x[spring->_cs[1] * 3 + 1],
			x[spring->_cs[0] * 3 + 2] - x[spring->_cs[1] * 3 + 2]
		);
		dir.SetNormalizedVector();
		dir *= 1.0e-3;

		int connectedNum1 = getAdjNum(spring, spring, if0, pivot);
		if (connectedNum0 + connectedNum1 == edgeNum)
			return;

		int np = addNode(x, px, masses, edges, p, pp);
		double mass = masses[pivot] * 0.5;
		masses[pivot] = masses[np] = mass;
		adjustVertex(faces, edges, spring, pivot, np);
	}
}
void Constraint::splitFace(VectorXi& faces, vector<double>& masses, EdgeBufferHost& edges, Spring* spring, int np, double w, TearingBuffer* order)
{
	int if0 = spring->_fs[0];
	int if1 = spring->_fs[1];
	int inspring0 = _springs.size();

	delEdge(edges, order->inodes[0], order->inodes[2]);

	faces[if0 * 3 + order->inodes_ord[0][1]] = np;
	_triangles[if0 * 3 + order->isprings_ord[0][2]] = inspring0 + 1;

	int inf0 = addFace(faces,
		np, order->inodes[2], order->inodes[1],
		inspring0, inspring0 + 1, order->isprings[2],
		order->inodes_ord[0], order->isprings_ord[0]);

	Vec3 nvec0 = spring->_vec * (1.0 - w);
	Vec3 nvec1 = _springs[order->isprings[2]]->getVecID(order->inodes[2]) + nvec0;

	spring->_vec *= w;
	spring->modifiyVertexID(1, np);

	if (if1 < 0) {
		addSpring(edges,
			np, order->inodes[2],
			order->inodes[1], -1,
			inf0, -1,
			spring->_length * (1.0 - w), spring->_vlength,
			nvec0, spring->_tearTheta);

		addSpring(edges,
			np, order->inodes[1],
			order->inodes[0], order->inodes[2],
			if0, inf0,
			nvec1.Length(), spring->_length,
			nvec1, spring->_tearTheta);

		masses[np] = (masses[order->inodes[0]] + masses[order->inodes[1]] + masses[order->inodes[2]]) * 0.25;
		masses[order->inodes[0]] *= 0.75;
		masses[order->inodes[1]] *= 0.75;
		masses[order->inodes[2]] *= 0.75;
	}
	else {
		faces[if1 * 3 + order->inodes_ord[1][1]] = np;
		_triangles[if1 * 3 + order->isprings_ord[1][2]] = inspring0 + 2;

		int inf1 = addFace(faces,
			np, order->inodes[2], order->inodes[3],
			inspring0, inspring0 + 2, order->isprings[3],
			order->inodes_ord[1], order->isprings_ord[1]);

		Vec3 nvec2 = _springs[order->isprings[3]]->getVecID(order->inodes[2]) + nvec0;

		addSpring(edges,
			np, order->inodes[2],
			order->inodes[1], order->inodes[3],
			inf0, inf1,
			spring->_length * (1.0 - w), spring->_vlength,
			nvec0, spring->_tearTheta);
		addSpring(edges,
			np, order->inodes[1],
			order->inodes[0], order->inodes[2],
			if0, inf0,
			nvec1.Length(), spring->_length,
			nvec1, spring->_tearTheta);
		addSpring(edges,
			np, order->inodes[3],
			order->inodes[0], order->inodes[2],
			if1, inf1,
			nvec2.Length(), spring->_length,
			nvec2, spring->_tearTheta);

		_springs[order->isprings[4]]->modifiyCrossV(order->inodes[2], np);
		_springs[order->isprings[3]]->modifiyCrossV(order->inodes[0], np);
		_springs[order->isprings[3]]->modifiyNbFace(if1, inf1);
		_springs[order->isprings[4]]->updateVlen(_triangles, _springs);
		_springs[order->isprings[3]]->updateVlen(_triangles, _springs);

		addEdge(edges, order->inodes[3], np);

		masses[np] = (masses[order->inodes[0]] + masses[order->inodes[1]] + masses[order->inodes[2]] + masses[order->inodes[3]]) * 0.2;
		masses[order->inodes[0]] *= 0.8;
		masses[order->inodes[1]] *= 0.8;
		masses[order->inodes[2]] *= 0.8;
		masses[order->inodes[3]] *= 0.8;
	}

	spring->_length *= w;

	_springs[order->isprings[1]]->modifiyCrossV(order->inodes[2], np);
	_springs[order->isprings[2]]->modifiyCrossV(order->inodes[0], np);
	_springs[order->isprings[2]]->modifiyNbFace(if0, inf0);
	_springs[order->isprings[1]]->updateVlen(_triangles, _springs);
	_springs[order->isprings[2]]->updateVlen(_triangles, _springs);

	addEdge(edges, order->inodes[0], np);
	addEdge(edges, order->inodes[1], np);
	addEdge(edges, order->inodes[2], np);
}
void Constraint::cutFace(
	VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
	Spring* spring, int np0, int np1, double w, TearingBuffer* order)
{
	int if0 = spring->_fs[0];
	int if1 = spring->_fs[1];
	int inspring0 = _springs.size();

	delEdge(edges, order->inodes[0], order->inodes[2]);

	faces[if0 * 3 + order->inodes_ord[0][1]] = np0;
	_triangles[if0 * 3 + order->isprings_ord[0][2]] = inspring0 + 1;

	int inf0 = addFace(faces,
		np1, order->inodes[2], order->inodes[1],
		inspring0, inspring0 + 2, order->isprings[2],
		order->inodes_ord[0], order->isprings_ord[0]);

	Vec3 nvec0 = spring->_vec * (1.0 - w);
	Vec3 nvec1 = _springs[order->isprings[2]]->getVecID(order->inodes[2]) + nvec0;

	spring->_vec *= w;
	spring->modifiyVertexID(1, np0);

	if (if1 < 0) {
		addSpring(edges,
			np1, order->inodes[2],
			order->inodes[1], -1,
			inf0, -1,
			spring->_length * (1.0 - w), spring->_vlength,
			nvec0, spring->_tearTheta);
		addSpring(edges,
			np0, order->inodes[1],
			order->inodes[0], -1,
			if0, -1,
			nvec1.Length(), (nvec1 * 0.5 + spring->_vec).Length(),
			nvec1, spring->_tearTheta);
		addSpring(edges,
			np1, order->inodes[1],
			order->inodes[2], -1,
			inf0, -1,
			nvec1.Length(), (nvec1 * 0.5 - nvec0).Length(),
			nvec1, spring->_tearTheta);

		masses[np0] = (masses[order->inodes[0]] + masses[order->inodes[1]] + masses[order->inodes[2]]) * 0.125;
		masses[np1] = masses[np0];
		masses[order->inodes[0]] *= 0.75;
		masses[order->inodes[1]] *= 0.75;
		masses[order->inodes[2]] *= 0.75;
	}
	else {
		faces[if1 * 3 + order->inodes_ord[1][1]] = np0;
		_triangles[if1 * 3 + order->isprings_ord[1][2]] = inspring0 + 4;

		int inf1 = addFace(faces,
			np1, order->inodes[2], order->inodes[3],
			inspring0, inspring0 + 3, order->isprings[3],
			order->inodes_ord[1], order->isprings_ord[1]);

		Vec3 nvec2 = _springs[order->isprings[3]]->getVecID(order->inodes[2]) + nvec0;

		addSpring(edges,
			np1, order->inodes[2],
			order->inodes[1], order->inodes[3],
			inf0, inf1,
			spring->_length * (1.0 - w), spring->_vlength,
			nvec0, spring->_tearTheta);
		addSpring(edges,
			np0, order->inodes[1],
			order->inodes[0], -1,
			if0, -1,
			nvec1.Length(), (nvec1 * 0.5 + spring->_vec).Length(),
			nvec1, spring->_tearTheta);
		addSpring(edges,
			np1, order->inodes[1],
			order->inodes[2], -1,
			inf0, -1,
			nvec1.Length(), (nvec1 * 0.5 - nvec0).Length(),
			nvec1, spring->_tearTheta);

		addSpring(edges,
			np1, order->inodes[3],
			order->inodes[2], -1,
			inf1, -1,
			nvec2.Length(), (nvec2 * 0.5 - nvec0).Length(),
			nvec2, spring->_tearTheta);
		addSpring(edges,
			np0, order->inodes[3],
			order->inodes[0], -1,
			if1, -1,
			nvec2.Length(), (nvec2 * 0.5 + spring->_vec).Length(),
			nvec2, spring->_tearTheta);

		_springs[order->isprings[3]]->modifiyNbFace(if1, inf1);
		_springs[order->isprings[3]]->modifiyCrossV(order->inodes[0], np1);
		_springs[order->isprings[4]]->modifiyCrossV(order->inodes[2], np0);
		_springs[order->isprings[3]]->updateVlen(_triangles, _springs);
		_springs[order->isprings[4]]->updateVlen(_triangles, _springs);

		addEdge(edges, order->inodes[3], np0);
		addEdge(edges, order->inodes[3], np1);

		masses[np0] = (masses[order->inodes[0]] + masses[order->inodes[1]] + masses[order->inodes[2]] + masses[order->inodes[3]]) * 0.2;
		masses[np1] = masses[np0];
		masses[order->inodes[0]] *= 0.8;
		masses[order->inodes[1]] *= 0.8;
		masses[order->inodes[2]] *= 0.8;
		masses[order->inodes[3]] *= 0.8;
	}

	spring->_length *= w;

	_springs[order->isprings[2]]->modifiyNbFace(if0, inf0);
	_springs[order->isprings[1]]->modifiyCrossV(order->inodes[2], np0);
	_springs[order->isprings[2]]->modifiyCrossV(order->inodes[0], np1);
	_springs[order->isprings[1]]->updateVlen(_triangles, _springs);
	_springs[order->isprings[2]]->updateVlen(_triangles, _springs);

	addEdge(edges, order->inodes[0], np0);
	addEdge(edges, order->inodes[1], np0);
	addEdge(edges, order->inodes[1], np1);
	addEdge(edges, order->inodes[2], np1);
}

int Constraint::getAdjNum(Spring* sbegin, Spring* send, int pastFace, int pivot) {
	Spring* ptr = sbegin;
	int num = 0;
	while (1) {
		int newFace = ptr->_fs[0] == pastFace ? ptr->_fs[1] : ptr->_fs[0];
		if (newFace < 0)
			break;
		for (int i = 0; i < 3; i++) {
			auto s = _springs[_triangles[newFace * 3 + i]];
			if (s != ptr && (s->_ps[0] == pivot || s->_ps[1] == pivot)) {
				ptr = s;
				break;
			}
		}
		if (ptr == send) return -num;
		num++;
		pastFace = newFace;
	}
	return num;
}
void Constraint::adjustVertex(VectorXi& faces, EdgeBufferHost& edges, Spring* spring, int pivot, int np) {
	Spring* ptr = spring, * tmp;
	int pastFace = ptr->_fs[1], newFace;
	while (1) {
		int inode = ptr->modifiyVertex(pivot, np) ^ 1;
		inode = ptr->_ps[inode];
		delEdge(edges, inode, pivot);
		addEdge(edges, inode, np);
		newFace = ptr->_fs[0] == pastFace ? ptr->_fs[1] : ptr->_fs[0];
		if (newFace < 0)
			break;
		for (int i = 0; i < 3; i++) {
			int ino = newFace * 3 + i;
			if (faces[ino] == pivot) {
				faces[ino] = np;
				break;
			}
		}
		for (int i = 0; i < 3; i++) {
			auto s = _springs[_triangles[newFace * 3 + i]];
			if (s == ptr) continue;
			if (s->_ps[0] == pivot || s->_ps[1] == pivot)
				tmp = s;
			else s->modifiyCrossV(pivot, np);
		}
		ptr = tmp;
		pastFace = newFace;
	}
	if (spring->_fs[1] < 0)
		return;

	ptr = spring;
	newFace = ptr->_fs[1];
	if (newFace < 0)
		return;

	for (int i = 0; i < 3; i++) {
		int ino = newFace * 3 + i;
		if (faces[ino] == pivot) {
			faces[ino] = np;
			break;
		}
	}
	for (int i = 0; i < 3; i++) {
		auto s = _springs[_triangles[newFace * 3 + i]];
		if (s == ptr) continue;
		if (s->_ps[0] == pivot || s->_ps[1] == pivot)
			tmp = s;
		else s->modifiyCrossV(pivot, np);
	}
	ptr = tmp;
	pastFace = newFace;
	while (1) {
		int inode = ptr->modifiyVertex(pivot, np) ^ 1;
		inode = ptr->_ps[inode];
		delEdge(edges, inode, pivot);
		addEdge(edges, inode, np);
		newFace = ptr->_fs[0] == pastFace ? ptr->_fs[1] : ptr->_fs[0];
		if (newFace < 0)
			break;
		for (int i = 0; i < 3; i++) {
			int ino = newFace * 3 + i;
			if (faces[ino] == pivot) {
				faces[ino] = np;
				break;
			}
		}
		for (int i = 0; i < 3; i++) {
			auto s = _springs[_triangles[newFace * 3 + i]];
			if (s == ptr) continue;
			if (s->_ps[0] == pivot || s->_ps[1] == pivot)
				tmp = s;
			else s->modifiyCrossV(pivot, np);
		}
		ptr = tmp;
		pastFace = newFace;
	}
}
void Constraint::adjustVertex(VectorXi& faces, EdgeBufferHost& edges, Spring* spring, int pastFace, int pivot, int np) {
	Spring* ptr = spring;
	while (1) {
		int inode = ptr->modifiyVertex(pivot, np) ^ 1;
		inode = ptr->_ps[inode];
		delEdge(edges, inode, pivot);
		addEdge(edges, inode, np);
		int newFace = ptr->_fs[0] == pastFace ? ptr->_fs[1] : ptr->_fs[0];
		if (newFace < 0)
			break;
		for (int i = 0; i < 3; i++) {
			int ino = newFace * 3 + i;
			if (faces[ino] == pivot) {
				faces[ino] = np;
				break;
			}
		}
		Spring* tmp;
		for (int i = 0; i < 3; i++) {
			auto s = _springs[_triangles[newFace * 3 + i]];
			if (s == ptr) continue;
			if (s->_ps[0] == pivot || s->_ps[1] == pivot)
				tmp = s;
			else s->modifiyCrossV(pivot, np);
		}
		ptr = tmp;
		pastFace = newFace;
	}
}
void Constraint::sliceHori(
	VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
	Spring* spring, double threshold, TearingBuffer* order)
{
	int if0 = spring->_fs[0];
	int if1 = spring->_fs[1];
	if (if1 < 0 || spring->_vlength < threshold) return;
	Vec3 ps[2];
	Vec3 pps[2];
	ps[0].Set(x[order->inodes[0] * 3 + 0], x[order->inodes[0] * 3 + 1], x[order->inodes[0] * 3 + 2]);
	ps[1].Set(x[order->inodes[2] * 3 + 0], x[order->inodes[2] * 3 + 1], x[order->inodes[2] * 3 + 2]);
	pps[0].Set(px[order->inodes[0] * 3 + 0], px[order->inodes[0] * 3 + 1], px[order->inodes[0] * 3 + 2]);
	pps[1].Set(px[order->inodes[2] * 3 + 0], px[order->inodes[2] * 3 + 1], px[order->inodes[2] * 3 + 2]);
	Vec3 dir(
		x[order->inodes[3] * 3 + 0] - x[order->inodes[1] * 3 + 0],
		x[order->inodes[3] * 3 + 1] - x[order->inodes[1] * 3 + 1],
		x[order->inodes[3] * 3 + 2] - x[order->inodes[1] * 3 + 2]
	);
	dir.SetNormalizedVector();
	dir *= 1.0e-3;

	int edgeNum[2] = { edges[order->inodes[0]].size() - 3, edges[order->inodes[2]].size() - 3 };
	bool apply = true;
	for (int n = 0; n < 2; n++) {
		int is0 = 1 + n, is1 = 4 - n;

		int pivot = order->inodes[n << 1];
		int connectedNum0 =
			getAdjNum(_springs[order->isprings[is0]], _springs[order->isprings[is1]], if0, pivot);
		if (connectedNum0 < 0)
			continue;

		apply = false;
		int connectedNum1 =
			getAdjNum(_springs[order->isprings[is1]], _springs[order->isprings[is1]], if1, pivot);

		if (connectedNum0 + connectedNum1 == edgeNum[n]) {
			Spring* ptr;
			int np, pastFace;
			if (connectedNum0 < connectedNum1) {
				np = addNode(x, px, masses, edges, ps[n] - dir, pps[n] - dir);
				ptr = _springs[order->isprings[is0]];
				pastFace = if0;

				if (spring->_fs[1] > -1) {
					int opinode = order->inodes[2 - (n << 1)];
					faces[if0 * 3 + order->inodes_ord[0][n]] = np;

					_triangles[if0 * 3 + order->isprings_ord[0][0]] = _springs.size();
					addSpring(edges,
						np, opinode,
						order->inodes[1], -1,
						if0, -1,
						spring->_length, (spring->_vec * 0.5 + _springs[order->isprings[2]]->getVecID(order->inodes[2])).Length(),
						-spring->getVecID(opinode), spring->_tearTheta);

					spring->deleteNbFace(if0);
					_springs[order->isprings[2 - n]]->modifiyCrossV(pivot, np);
					spring->updateVlen(_triangles, _springs);

					addEdge(edges, opinode, np);
				}
				else {
					int opinode = faces[if0 * 3 + order->inodes_ord[0][0]];
					faces[if0 * 3 + order->inodes_ord[0][1]] = np;

					_springs[_triangles[if0 * 3 + order->isprings_ord[0][0]]]->modifiyVertex(
						pivot, np);
					_springs[order->isprings[1]]->modifiyCrossV(pivot, np);
					delEdge(edges, opinode, pivot);
					addEdge(edges, opinode, np);
				}
			}
			else {
				np = addNode(x, px, masses, edges, ps[n] + dir, pps[n] + dir);
				ptr = _springs[order->isprings[is1]];
				pastFace = if1;

				if (spring->_fs[1] > -1) {
					int opinode = order->inodes[2 - (n << 1)];
					faces[if1 * 3 + order->inodes_ord[1][n]] = np;

					_triangles[if1 * 3 + order->isprings_ord[1][0]] = _springs.size();
					addSpring(edges,
						np, opinode,
						order->inodes[3], -1,
						if1, -1,
						spring->_length, (spring->_vec * 0.5 + _springs[order->isprings[3]]->getVecID(order->inodes[2])).Length(),
						-spring->getVecID(opinode), spring->_tearTheta);

					spring->deleteNbFace(if1);
					_springs[order->isprings[3 + n]]->modifiyCrossV(pivot, np);
					spring->updateVlen(_triangles, _springs);

					addEdge(edges, opinode, np);
				}
				else {
					int opinode = faces[if1 * 3 + order->inodes_ord[1][0]];
					faces[if1 * 3 + order->inodes_ord[1][1]] = np;

					_springs[_triangles[if1 * 3 + order->isprings_ord[1][0]]]->modifiyVertex(
						pivot, np);
					_springs[order->isprings[4]]->modifiyCrossV(pivot, np);
					delEdge(edges, opinode, pivot);
					addEdge(edges, opinode, np);
				}
			}
			masses[np] = masses[order->inodes[n << 1]] * 0.5;
			masses[order->inodes[n << 1]] = masses[np];
			adjustVertex(faces, edges, ptr, pastFace, pivot, np);
		}
		else {
			int np0 = addNode(x, px, masses, edges, ps[n] - dir, pps[n] - dir);
			int np1 = addNode(x, px, masses, edges, ps[n] + dir, pps[n] + dir);
			if (spring->_fs[1] > -1) {
				int opinode = order->inodes[2 - (n << 1)];
				faces[if0 * 3 + order->inodes_ord[0][n]] = np0;
				faces[if1 * 3 + order->inodes_ord[1][n]] = np1;

				_triangles[if0 * 3 + order->isprings_ord[0][0]] = _springs.size();
				addSpring(edges,
					np0, opinode,
					order->inodes[1], -1,
					if0, -1,
					spring->_length, (spring->_vec * 0.5 + _springs[order->isprings[2]]->getVecID(order->inodes[2])).Length(),
					-spring->getVecID(opinode), spring->_tearTheta);

				_springs[order->isprings[2 - n]]->modifiyCrossV(pivot, np0);
				_springs[order->isprings[3 + n]]->modifiyCrossV(pivot, np1);
				spring->modifiyVertexID(n, np1);
				spring->deleteNbFace(if0);
				spring->updateVlen(_triangles, _springs);
				
				delEdge(edges, opinode, pivot);
				addEdge(edges, opinode, np0);
				addEdge(edges, opinode, np1);
			}
			else {
				int opinode0 = faces[if0 * 3 + order->inodes_ord[0][0]];
				int opinode1 = faces[if1 * 3 + order->inodes_ord[1][0]];
				faces[if0 * 3 + order->inodes_ord[0][1]] = np0;
				faces[if1 * 3 + order->inodes_ord[1][1]] = np1;

				_springs[_triangles[if0 * 3 + order->isprings_ord[0][0]]]->modifiyVertex(
					pivot, np0);
				_springs[_triangles[if1 * 3 + order->isprings_ord[1][0]]]->modifiyVertex(
					pivot, np1);

				_springs[order->isprings[1]]->modifiyCrossV(pivot, np0);
				_springs[order->isprings[4]]->modifiyCrossV(pivot, np1);

				delEdge(edges, opinode0, pivot);
				delEdge(edges, opinode1, pivot);
				addEdge(edges, opinode0, np0);
				addEdge(edges, opinode1, np1);
			}
			masses[np0] = masses[np1] = masses[order->inodes[n << 1]] / 3.0;
			masses[order->inodes[n << 1]] -= masses[np0] + masses[np1];
			adjustVertex(faces, edges, _springs[order->isprings[is0]], if0, pivot, np0);
			adjustVertex(faces, edges, _springs[order->isprings[is1]], if1, pivot, np1);
		}
	}

	if (apply) {
		if (spring->_length < threshold) return;
		ps[0] = (ps[0] + ps[1]) * 0.5;
		pps[0] = (pps[0] + pps[1]) * 0.5;
		int inf0 = faces.size() / 3;
		int inf1 = inf0 + 1;
		int inspring0 = _springs.size();
		int np = addNode(x, px, masses, edges, ps[0], pps[0]);
		splitFace(faces, masses, edges, spring, np, 0.5, order);
		addNode(x, px, masses, edges, ps[0] + dir, pps[0] + dir);

		faces[if1 * 3 + order->inodes_ord[1][1]] = np + 1;
		faces[inf1 * 3 + order->inodes_ord[1][0]] = np + 1;
		_triangles[if1 * 3 + order->isprings_ord[1][0]] = inspring0 + 3;
		_triangles[inf1 * 3 + order->isprings_ord[1][0]] = inspring0 + 4;

		addSpring(edges,
			order->inodes[0], np + 1,
			order->inodes[3], -1,
			if1, -1,
			spring->_length, (spring->_vec * 0.5 + _springs[order->isprings[4]]->getVecID(order->inodes[0])).Length(),
			spring->_vec, spring->_tearTheta);
		addSpring(edges,
			np + 1, order->inodes[2],
			order->inodes[3], -1,
			inf1, -1,
			_springs[inspring0]->_length, (_springs[inspring0]->_vec * 0.5 - _springs[order->isprings[3]]->getVecID(order->inodes[2])).Length(),
			_springs[inspring0]->_vec, spring->_tearTheta);

		spring->deleteNbFace(if1);
		_springs[inspring0]->deleteNbFace(inf1);
		_springs[inspring0 + 2]->modifiyVertex(np, np + 1);
		_springs[order->isprings[4]]->modifiyCrossV(np, np + 1);
		_springs[order->isprings[3]]->modifiyCrossV(np, np + 1);

		spring->updateVlen(_triangles, _springs);
		_springs[inspring0]->updateVlen(_triangles, _springs);

		delEdge(edges, order->inodes[3], np);
		addEdge(edges, order->inodes[0], np + 1);
		addEdge(edges, order->inodes[2], np + 1);
		addEdge(edges, order->inodes[3], np + 1);

		masses[np] *= 0.5;
		masses[np + 1] = masses[np];
	}
	cleanup(faces, x, px, masses, edges, _springs[order->isprings[1]], order->inodes[1]);
	cleanup(faces, x, px, masses, edges, _springs[order->isprings[4]], order->inodes[3]);
}
void Constraint::sliceHori(
	VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
	int ispring, double threshold)
{
	TearingBuffer order;
	order.getOrders(faces, _triangles, _springs, ispring);
	sliceHori(faces, x, px, masses, edges, _springs[ispring], threshold, &order);
}
void Constraint::sliceVert(
	VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
	Spring* spring, double threshold, TearingBuffer* order)
{
	int if0 = spring->_fs[0];
	int if1 = spring->_fs[1];
	if (if1 < 0 || spring->_length < threshold) return;

	double w;
	Vec3 vp(
		x[order->inodes[2] * 3 + 0] - x[order->inodes[0] * 3 + 0],
		x[order->inodes[2] * 3 + 1] - x[order->inodes[0] * 3 + 1],
		x[order->inodes[2] * 3 + 2] - x[order->inodes[0] * 3 + 2]);
	Vec3 vq(
		x[order->inodes[3] * 3 + 0] - x[order->inodes[1] * 3 + 0],
		x[order->inodes[3] * 3 + 1] - x[order->inodes[1] * 3 + 1],
		x[order->inodes[3] * 3 + 2] - x[order->inodes[1] * 3 + 2]);
	double t0 = Dot(vp, vp);
	double t1 = Dot(vq, vq);
	double t2 = Dot(vp, vq);
	double det = t0 * t1 - t2 * t2;
	if (fabs(det) < 1.0e-20)
		w = 0.5;
	else {
		Vec3 pq(
			x[order->inodes[1] * 3 + 0] - x[order->inodes[0] * 3 + 0],
			x[order->inodes[1] * 3 + 1] - x[order->inodes[0] * 3 + 1],
			x[order->inodes[1] * 3 + 2] - x[order->inodes[0] * 3 + 2]);
		double t3 = Dot(vp, pq);
		double t4 = Dot(vq, pq);
		double invdet = 1.0 / det;
		w = (+t1 * t3 - t2 * t4) * invdet;
	}
	//printf("%f\n", w);
	//double w = 0.5;
	if (spring->_length * w < threshold || spring->_length * (1.0 - w) < threshold)
		return;
	Vec3 cen(
		x[order->inodes[0] * 3 + 0] + (x[order->inodes[2] * 3 + 0] - x[order->inodes[0] * 3 + 0]) * w,
		x[order->inodes[0] * 3 + 1] + (x[order->inodes[2] * 3 + 1] - x[order->inodes[0] * 3 + 1]) * w,
		x[order->inodes[0] * 3 + 2] + (x[order->inodes[2] * 3 + 2] - x[order->inodes[0] * 3 + 2]) * w);
	Vec3 pcen(
		px[order->inodes[0] * 3 + 0] + (px[order->inodes[2] * 3 + 0] - px[order->inodes[0] * 3 + 0]) * w,
		px[order->inodes[0] * 3 + 1] + (px[order->inodes[2] * 3 + 1] - px[order->inodes[0] * 3 + 1]) * w,
		px[order->inodes[0] * 3 + 2] + (px[order->inodes[2] * 3 + 2] - px[order->inodes[0] * 3 + 2]) * w);

	vp.SetNormalizedVector();
	vp *= 1.0e-3;

	int inf0 = faces.size() / 3;
	int inspring0 = _springs.size();
	int np = addNode(x, px, masses, edges, cen, pcen);
	addNode(x, px, masses, edges, cen + vp, pcen + vp);
	cutFace(faces, x, px, masses, edges, spring, np, np + 1, w, order);

	cleanup(faces, x, px, masses, edges, _springs[inspring0 + 1], order->inodes[1]);
	cleanup(faces, x, px, masses, edges, _springs[inspring0 + 2], order->inodes[1]);
	cleanup(faces, x, px, masses, edges, _springs[inspring0 + 3], order->inodes[3]);
	cleanup(faces, x, px, masses, edges, _springs[inspring0 + 4], order->inodes[3]);

	cleanup(faces, x, px, masses, edges, spring, order->inodes[0]);
	cleanup(faces, x, px, masses, edges, _springs[inspring0], order->inodes[2]);
}
void Constraint::sliceVert(
	VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges,
	int ispring, double threshold)
{
	TearingBuffer order;
	order.getOrders(faces, _triangles, _springs, ispring);
	sliceVert(faces, x, px, masses, edges, _springs[ispring], threshold, &order);
}
bool Constraint::getSliceRatio(
	const Vec3& grad, const Vec3* ps, double threshold,
	const TearingBuffer* order, int* regions, double* ratio)
{
	Vec3 pc, p[2], dir;
	Vec3 cen = ps[regions[0] >> 1];
	double pCenDot, pDot;
	double _ratio;

	int isprings[2], ino;
	isprings[0] = ((regions[0] + 3) & 7) + 1 >> 1;
	isprings[1] = ((isprings[0] + 2) & 7) + 1 >> 1;
	ino = (regions[0] + 4) & 7;
	pc = ps[ino >> 1];
	p[0] = ps[((regions[0] + 2) & 7) >> 1];
	p[1] = ps[((regions[0] + 6) & 7) >> 1];
	pCenDot = Dot(cen - pc, grad);
	for (int i = 0; i < 2; i++) {
		pDot = Dot(p[i] - pc, grad);
		_ratio = pCenDot / pDot;
		if (_ratio >= -1.0e-5 && _ratio <= 1.0 + 1.0e-5) {
			double len = _springs[isprings[i]]->_length;
			if (len * _ratio < threshold && _ratio < 0.5) {
				regions[1] = ino;
			}
			else if (len * (1.0 - _ratio) < threshold) {
				regions[1] = ino - 2 + (i << 2);
			}
			else {
				regions[1] = ino - 1 + (i << 1);
				*ratio = _ratio;
			}
			break;
		}
		if (i == 1)
			return false;
	}
	if (regions[1] < 0)
		regions[1] += 8;
	else
		regions[1] = regions[1] & 7;
	return true;
}
bool Constraint::getSliceRatio3(
	const Vec3& grad, const Vec3* ps, double threshold,
	const TearingBuffer* order, int* regions, double* ratio)
{
	Vec3 p0, p1, cen;
	double pCenDot, pDot;
	double _ratio, len;
	if (regions[0] == 1) {
		cen = ps[0];
		p0 = ps[1];
		p1 = ps[2];
		len = _springs[order->isprings[2]]->_length;
		regions[1] = 4;
	}
	else if (regions[0] == 3) {
		cen = ps[1];
		p0 = ps[2];
		p1 = ps[0];
		len = _springs[order->isprings[0]]->_length;
		regions[1] = 0;
	}
	else {
		cen = ps[2];
		p0 = ps[0];
		p1 = ps[1];
		len = _springs[order->isprings[1]]->_length;
		regions[1] = 2;
	}
	pCenDot = Dot(cen - p0, grad);
	pDot = Dot(p1 - p0, grad);
	_ratio = pCenDot / pDot;
	if (_ratio >= -1.0e-5 && _ratio <= 1.0 + 1.0e-5) {
		if (len * _ratio < threshold && _ratio < 0.5) {
			regions[1]--;
			if (regions[1] < 0)
				regions[1] = 5;
		}
		else if (len * (1.0 - _ratio) < threshold) {
			regions[1]++;
		}
		else *ratio = _ratio;
		return true;
	}
	return false;
}

double Constraint::weaklyVertex(VectorXd& x, Spring* spring0, Spring* spring1, int pastFace0, int pastFace1, int pivot, const Vec3& grad) {
	if (spring0->_fs[1] == -1 || spring1->_fs[1] == -1)
		return -1.0;
	Spring* ptr = spring0;
	int pastFace = pastFace0;
	Vec3 v0, v1;
	while (1) {
		int newFace = ptr->_fs[0] == pastFace ? ptr->_fs[1] : ptr->_fs[0];
		if (newFace < 0) {
			int ino0 = pivot;
			int ino1 = ptr->_ps[0] == pivot ? ptr->_ps[1] : ptr->_ps[0];
			v0.Set(
				x[ino1 * 3 + 0] - x[ino0 * 3 + 0],
				x[ino1 * 3 + 1] - x[ino0 * 3 + 1],
				x[ino1 * 3 + 2] - x[ino0 * 3 + 2]);
			v0.SetNormalizedVector();
			break;
		}
		for (int i = 0; i < 3; i++) {
			auto s = _springs[_triangles[newFace * 3 + i]];
			if (s != ptr && (s->_ps[0] == pivot || s->_ps[1] == pivot)) {
				ptr = s;
				break;
			}
		}
		if (ptr == spring1)
			return -1.0;
		pastFace = newFace;
	}
	ptr = spring1;
	pastFace = pastFace1;
	while (1) {
		int newFace = ptr->_fs[0] == pastFace ? ptr->_fs[1] : ptr->_fs[0];
		if (newFace < 0) {
			int ino0 = pivot;
			int ino1 = ptr->_ps[0] == pivot ? ptr->_ps[1] : ptr->_ps[0];
			v1.Set(
				x[ino1 * 3 + 0] - x[ino0 * 3 + 0],
				x[ino1 * 3 + 1] - x[ino0 * 3 + 1],
				x[ino1 * 3 + 2] - x[ino0 * 3 + 2]);
			v1.SetNormalizedVector();
			break;
		}
		for (int i = 0; i < 3; i++) {
			auto s = _springs[_triangles[newFace * 3 + i]];
			if (s != ptr && (s->_ps[0] == pivot || s->_ps[1] == pivot)) {
				ptr = s;
				break;
			}
		}
		pastFace = newFace;
	}
	v0 = (v0 + v1) * 0.5;
	return LengthSquared(Cross(v0, grad));
}
void Constraint::tearing(VectorXi& faces, VectorXd& x, VectorXd& px, vector<double>& masses, EdgeBufferHost& edges) {
	//printf("%d\n", _springs.size());
	//printf("%d\n", edges.size());
	double threshold = 0.01;
	int springSize = _springs.size();

	TearingBuffer order;
	Spring* spring, *ptr;
	Vec3 p[4], pp[4];
	//if (_springs.size() > 3418)return;
	for (int i = 0; i < springSize; i++)
		_springs[i]->_flag = false;

	for (int i = 0; i < springSize; i++) {
		//if (i != 3227) continue;
		spring = _springs[i];
		if (spring->_flag || spring->_length < threshold && spring->_vlength < threshold)
			continue;
		order.getOrders(faces, _triangles, _springs, i);
		p[0].Set(x[order.inodes[0] * 3 + 0], x[order.inodes[0] * 3 + 1], x[order.inodes[0] * 3 + 2]);
		p[1].Set(x[order.inodes[1] * 3 + 0], x[order.inodes[1] * 3 + 1], x[order.inodes[1] * 3 + 2]);
		p[2].Set(x[order.inodes[2] * 3 + 0], x[order.inodes[2] * 3 + 1], x[order.inodes[2] * 3 + 2]);
		pp[0].Set(px[order.inodes[0] * 3 + 0], px[order.inodes[0] * 3 + 1], px[order.inodes[0] * 3 + 2]);
		pp[1].Set(px[order.inodes[1] * 3 + 0], px[order.inodes[1] * 3 + 1], px[order.inodes[1] * 3 + 2]);
		pp[2].Set(px[order.inodes[2] * 3 + 0], px[order.inodes[2] * 3 + 1], px[order.inodes[2] * 3 + 2]);

		if (spring->_fs[1] > -1) {
			p[3].Set(x[order.inodes[3] * 3 + 0], x[order.inodes[3] * 3 + 1], x[order.inodes[3] * 3 + 2]);
			pp[3].Set(px[order.inodes[3] * 3 + 0], px[order.inodes[3] * 3 + 1], px[order.inodes[3] * 3 + 2]);
			double ratio0 = spring->_length > threshold ? (p[0] - p[2]).Length() / spring->_length : 0;
			double ratio1 = spring->_vlength > threshold ? (p[1] - p[3]).Length() / spring->_vlength : 0;

			if (ratio0 >= spring->_tearTheta || ratio1 >= spring->_tearTheta)
			{
				Vec3 cen = (p[0] + p[2]) * 0.5;
				/*Vec3 grad = (ratio0 < ratio1) ?
					p[1] + p[2] + p[3] - pp[1] - pp[2] - pp[3] - 3.0 * (p[0] - pp[0]) :
					p[0] + p[2] + p[3] - pp[0] - pp[2] - pp[3] - 3.0 * (p[1] - pp[1]);*/
				Vec3 grad =
					p[1] + p[2] + p[3] - pp[1] - pp[2] - pp[3] - 3.0 * (p[0] - pp[0])
					+ p[0] + p[2] + p[3] - pp[0] - pp[2] - pp[3] - 3.0 * (p[1] - pp[1]);
				grad.SetNormalizedVector();

				int regions[2];
				double ratio;
				{
					double maxvw = -DBL_MAX, tmp;
					regions[0] = -1;
					bool gradDot0 = Dot(grad, p[2] - p[0]) > 0;
					bool gradDot1 = Dot(grad, p[1] - p[0]) > 0;
					bool gradDot2 = Dot(grad, p[1] - p[2]) > 0;
					bool gradDot3 = Dot(grad, p[3] - p[2]) > 0;
					bool gradDot4 = Dot(grad, p[3] - p[0]) > 0;
					if (gradDot0 != gradDot1 || gradDot0 != gradDot4) {
						maxvw = weaklyVertex(x, _springs[order.isprings[1]], _springs[order.isprings[4]], spring->_fs[0], spring->_fs[1], spring->_ps[0], grad);
						regions[0] = 0;
					}
					if (gradDot1 != gradDot2) {
						tmp = weaklyVertex(x, _springs[order.isprings[1]], _springs[order.isprings[2]], spring->_fs[0], spring->_fs[0], spring->_cs[0], grad);
						if (maxvw < tmp) {
							maxvw = tmp;
							regions[0] = 2;
						}
					}
					if (gradDot0 == gradDot2 || gradDot0 == gradDot3) {
						tmp = weaklyVertex(x, _springs[order.isprings[2]], _springs[order.isprings[3]], spring->_fs[0], spring->_fs[1], spring->_ps[1], grad);
						if (maxvw < tmp) {
							maxvw = tmp;
							regions[0] = 4;
						}
					}
					if (gradDot3 != gradDot4) {
						tmp = weaklyVertex(x, _springs[order.isprings[3]], _springs[order.isprings[4]], spring->_fs[1], spring->_fs[1], spring->_cs[1], grad);
						if (maxvw < tmp) {
							maxvw = tmp;
							regions[0] = 6;
						}
					}
					if (regions[0] == -1)
						continue;
				}

				if (!getSliceRatio(grad, p, threshold, &order, regions, &ratio))
					continue;

				if (abs(regions[0] - regions[1]) == 2 || abs(regions[0] - regions[1]) == 6) {
					int ino = order.isprings[(regions[0] >> 1) + 1];
					order.getOrders(faces, _triangles, _springs, ino);
					sliceHori(faces, x, px, masses, edges, _springs[ino], threshold, &order);
				}
				else if (regions[0] + regions[1] == 4) {
					sliceHori(faces, x, px, masses, edges, spring, threshold, &order);
				}
				else if (regions[0] + regions[1] == 8) {
					sliceVert(faces, x, px, masses, edges, spring, threshold, &order);
				}
				else if (regions[0] == 0 || regions[0] == 4) {
					int ino = order.isprings[regions[1] + 1 >> 1];
					if (_springs[ino]->_ps[0] != order.inodes[((regions[0] + 4) & 7) >> 1])
						ratio = 1.0 - ratio;
					if (_springs[ino]->_length < threshold)
						continue;
					order.getOrders(faces, _triangles, _springs, ino);
					p[0].Set(x[order.inodes[0] * 3 + 0], x[order.inodes[0] * 3 + 1], x[order.inodes[0] * 3 + 2]);
					p[1].Set(x[order.inodes[2] * 3 + 0], x[order.inodes[2] * 3 + 1], x[order.inodes[2] * 3 + 2]);
					pp[0].Set(px[order.inodes[0] * 3 + 0], px[order.inodes[0] * 3 + 1], px[order.inodes[0] * 3 + 2]);
					pp[1].Set(px[order.inodes[2] * 3 + 0], px[order.inodes[2] * 3 + 1], px[order.inodes[2] * 3 + 2]);
					int np = addNode(x, px, masses, edges,
						p[0] + (p[1] - p[0]) * ratio, pp[0] + (pp[1] - pp[0]) * ratio);
					splitFace(faces, masses, edges, _springs[ino], np, ratio, &order);

					if (_springs[ino]->_fs[1] > -1) {
						int inspring = _springs.size() - 2;
						if (_springs[inspring]->_fs[0] == spring->_fs[0] || _springs[inspring]->_fs[1] == spring->_fs[0] ||
							_springs[inspring]->_fs[0] == spring->_fs[1] || _springs[inspring]->_fs[1] == spring->_fs[1]) {
							order.getOrders(faces, _triangles, _springs, inspring);
							sliceHori(faces, x, px, masses, edges, _springs[inspring], threshold, &order);
						}
						else {
							order.getOrders(faces, _triangles, _springs, inspring + 1);
							sliceHori(faces, x, px, masses, edges, _springs[inspring + 1], threshold, &order);
						}
					}
					else {
						int inspring = _springs.size() - 1;
						order.getOrders(faces, _triangles, _springs, inspring);
						sliceHori(faces, x, px, masses, edges, _springs[inspring], threshold, &order);
					}
				}
				else {
					int ino = order.isprings[regions[1] + 1 >> 1];
					if (_springs[ino]->_ps[0] != order.inodes[((regions[0] + 4) & 7) >> 1])
						ratio = 1.0 - ratio;
					if (_springs[ino]->_length < threshold)
						continue;
					order.getOrders(faces, _triangles, _springs, ino);
					p[0].Set(x[order.inodes[0] * 3 + 0], x[order.inodes[0] * 3 + 1], x[order.inodes[0] * 3 + 2]);
					p[1].Set(x[order.inodes[2] * 3 + 0], x[order.inodes[2] * 3 + 1], x[order.inodes[2] * 3 + 2]);
					pp[0].Set(px[order.inodes[0] * 3 + 0], px[order.inodes[0] * 3 + 1], px[order.inodes[0] * 3 + 2]);
					pp[1].Set(px[order.inodes[2] * 3 + 0], px[order.inodes[2] * 3 + 1], px[order.inodes[2] * 3 + 2]);
					int np = addNode(x, px, masses, edges,
						p[0] + (p[1] - p[0]) * ratio, pp[0] + (pp[1] - pp[0]) * ratio);
					splitFace(faces, masses, edges, _springs[ino], np, ratio, &order);

					order.getOrders(faces, _triangles, _springs, i);
					sliceVert(faces, x, px, masses, edges, spring, threshold, &order);
				}
			}
		}
		else {
			double ratio0 = spring->_length > threshold ? (p[0] - p[2]).Length() / spring->_length : 0;
			double ratio1 = spring->_vlength > threshold ? (p[1] - (p[0] + p[1]) * 0.5).Length() / spring->_vlength : 0;
			if (ratio0 >= spring->_tearTheta || ratio1 >= spring->_tearTheta)
			{
				Vec3 cen = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
				Vec3 grad =
					p[1] + p[2] - pp[1] - pp[2] - 2.0 * (p[0] - pp[0])
					+ p[0] + p[2] - pp[0] - pp[2] - 2.0 * (p[1] - pp[1]);
				grad.SetNormalizedVector();

				int regions[2];
				double ratio;
				{
					double maxvw = -DBL_MAX, tmp;
					regions[0] = -1;
					bool gradDot0 = Dot(grad, p[2] - p[0]) > 0;
					bool gradDot1 = Dot(grad, p[1] - p[0]) > 0;
					bool gradDot2 = Dot(grad, p[1] - p[2]) > 0;
					if (gradDot0 != gradDot1) {
						maxvw = weaklyVertex(x, _springs[order.isprings[0]], _springs[order.isprings[1]], spring->_fs[0], spring->_fs[0], spring->_ps[0], grad);

						regions[0] = 1;
					}
					if (gradDot1 != gradDot2) {
						tmp = weaklyVertex(x, _springs[order.isprings[1]], _springs[order.isprings[2]], spring->_fs[0], spring->_fs[0], spring->_cs[0], grad);
						if (maxvw < tmp) {
							maxvw = tmp;
							regions[0] = 3;
						}
					}
					if (gradDot0 == gradDot2) {
						tmp = weaklyVertex(x, _springs[order.isprings[2]], _springs[order.isprings[0]], spring->_fs[0], spring->_fs[0], spring->_ps[1], grad);
						if (maxvw < tmp) {
							maxvw = tmp;
							regions[0] = 5;
						}
					}
					if (regions[0] == -1)
						continue;
				}

				if (!getSliceRatio3(grad, p, threshold, &order, regions, &ratio))
					continue;

				if (regions[0] + regions[1] == 4) {
					order.getOrders(faces, _triangles, _springs, order.isprings[1]);
					sliceHori(faces, x, px, masses, edges, _springs[order.isprings[0]], threshold, &order);
				}
				else if (regions[0] + regions[1] == 6) {
					sliceHori(faces, x, px, masses, edges, spring, threshold, &order);
				}
				else if (regions[0] + regions[1] == 8) {
					order.getOrders(faces, _triangles, _springs, order.isprings[2]);
					sliceHori(faces, x, px, masses, edges, _springs[order.isprings[0]], threshold, &order);
				}
				else {
					int ino = order.isprings[regions[1] >> 1];
					if (_springs[ino]->_ps[1] != order.inodes[regions[1] >> 1])
						ratio = 1.0 - ratio;
					if (_springs[ino]->_length < threshold)
						continue;
					order.getOrders(faces, _triangles, _springs, ino);
					p[0].Set(x[order.inodes[0] * 3 + 0], x[order.inodes[0] * 3 + 1], x[order.inodes[0] * 3 + 2]);
					p[1].Set(x[order.inodes[2] * 3 + 0], x[order.inodes[2] * 3 + 1], x[order.inodes[2] * 3 + 2]);
					pp[0].Set(px[order.inodes[0] * 3 + 0], px[order.inodes[0] * 3 + 1], px[order.inodes[0] * 3 + 2]);
					pp[1].Set(px[order.inodes[2] * 3 + 0], px[order.inodes[2] * 3 + 1], px[order.inodes[2] * 3 + 2]);
					int np = addNode(x, px, masses, edges,
						p[0] + (p[1] - p[0]) * ratio, pp[0] + (pp[1] - pp[0]) * ratio);
					splitFace(faces, masses, edges, _springs[ino], np, ratio, &order);

					if (_springs[ino]->_fs[1] > -1) {
						int inspring = _springs.size() - 2;
						if (_springs[inspring]->_fs[0] == spring->_fs[0] || _springs[inspring]->_fs[1] == spring->_fs[0] ||
							_springs[inspring]->_fs[0] == spring->_fs[1] || _springs[inspring]->_fs[1] == spring->_fs[1]) {
							order.getOrders(faces, _triangles, _springs, inspring);
							sliceHori(faces, x, px, masses, edges, _springs[inspring], threshold, &order);
						}
						else {
							order.getOrders(faces, _triangles, _springs, inspring + 1);
							sliceHori(faces, x, px, masses, edges, _springs[inspring + 1], threshold, &order);
						}
					}
					else {
						int inspring = _springs.size() - 1;
						order.getOrders(faces, _triangles, _springs, inspring);
						sliceHori(faces, x, px, masses, edges, _springs[inspring], threshold, &order);
					}
				}
			}
		}
	}
}