#include "Obstacle.h"

Obstacle::Obstacle(void)
{
}

Obstacle::Obstacle(Mesh* mesh, double x, double y, double z, double w, double size, bool apply_scale)
{
	_mesh = mesh;
	_pos.x = x;
	_pos.y = y;
	_pos.z = z;
	_w = w;
	_size = size;
	init(apply_scale);
	initBVH();
	computeNormal();
	//_zv = 0.8;
	_zv = 1.8;

	//_zv = 0;
	//_za = 0.1;
}

Obstacle::~Obstacle(void)
{
}

void Obstacle::scale(double size)
{
	auto mean_node = _nodes.colwise().mean();
	_nodes.rowwise() -= mean_node;
	auto w = size / (_nodes.maxCoeff() - _nodes.minCoeff());
	_nodes.array() *= w;
}

void Obstacle::init(bool apply_scale)
{
	vector<Eigen::RowVector3d> positions;
	vector<Eigen::RowVector3i> faces;

	for (auto v : _mesh->_vertices) {
		positions.push_back(Eigen::RowVector3d{ v->x(), v->y(), v->z() });
	}

	for (auto f : _mesh->_faces) {
		faces.push_back(Eigen::RowVector3i{ f->v(0)->_index, f->v(1)->_index, f->v(2)->_index });
	}

	_nodes = Eigen::MatrixXd(positions.size(), 3);
	_faces = Eigen::MatrixXi(faces.size(), 3);

	for (int i = 0; i < (int)positions.size(); i++) {
		_nodes.row(i) = positions[i];
	}

	for (int i = 0; i < (int)faces.size(); i++) {
		_faces.row(i) = faces[i];
	}

	_ms = Eigen::VectorXd(_nodes.rows()); _ms.setOnes();
	_velocities = Eigen::MatrixX3d(_nodes.rows(), _nodes.cols()); _velocities.setZero();
	_fixedPoints.resize(_nodes.rows(), true);

	double massPerParticle = 1.0;
	for (int i = 0; i < _ms.rows(); i++) {
		if (isFixed(i)) {
			continue;
		}
		auto const eq = [](double const a, double const b) {
			double constexpr eps = 1e-5;
			double const diff = abs(a - b);
			return diff <= eps;
		};
		if (!eq(_ms(i), massPerParticle)) {
			_ms(i) = massPerParticle;
		}
	}
	if (apply_scale) {
		scale(_size);
	}
	for (int i = 0; i < _nodes.rows(); i++) {
		_nodes.row(i)[0] += _pos.x;
		_nodes.row(i)[1] += _pos.y;
		_nodes.row(i)[2] += _pos.z;
		//_nodes.row(i)[2] += 0.12f;
	}
	_normals = Eigen::MatrixXf::Zero(3, _mesh->_faces.size());
}
void Obstacle::initBVH(void)
{
	ParamManager::initObjectParamHost(h_objParams, _faces.rows(), _nodes.rows(), true);
	_pos0.resize(_faces.rows() * 3);
	_masses.resize(_nodes.rows());
	vector< vector<int> > nbfaces;
	int nodeSize = _nodes.rows();
	nbfaces.resize(nodeSize);

	h_nbFaces.index = (int*)malloc((nodeSize + 1) * sizeof(int));
	h_nbFaces.index[0] = 0;

	for (int i = 0; i < _faces.rows(); i++) {
		for (int j = 0; j < 3; j++) {
			int ino = _faces(i, j);
			h_objParams.faces[i * 3 + j] = ino;
			nbfaces[ino].push_back(i);
			h_nbFaces.index[ino + 1] = nbfaces[ino].size();
		}
	}
	for (int i = 1; i < nodeSize; i++)
		h_nbFaces.index[i + 1] += h_nbFaces.index[i];

	h_nbFaces.array = (int*)malloc(h_nbFaces.index[nodeSize] * sizeof(int));
	for (int i = 0; i < nodeSize; i++) {
		int ino = 0;

		for (int j = 0; j < nbfaces[i].size(); j++)
			h_nbFaces.array[h_nbFaces.index[i] + j] = nbfaces[i][j];

		for (int j = 0; j < 3; j++)
			h_objParams.vertices[i * 3 + j] = _nodes(i, j);
		h_objParams.masses[i] = m(i);
		_masses[i] = m(i);
	}

	//_bvh = new BVH(h_objParams, h_nbFaces);
	_bvhHost = new BVHHost(h_objParams, h_nbFaces);
	//CUDA_CHECK(cudaFreeHost(h_nbFaces.index));
	//CUDA_CHECK(cudaFreeHost(h_nbFaces.array));
}
inline Eigen::RowVector3d rotateY(Eigen::RowVector3d v, double degree) {
	double radian = degree * 3.141592 / 180.0;
	double c = cos(radian);
	double s = sin(radian);
	Eigen::RowVector3d nv = v;
	nv[0] = v[0] * c - v[2] * s;
	nv[2] = v[0] * s + v[2] * c;
	return nv;
}
void Obstacle::update(double dt) {
	/*if (fabs(_zv + _za) > 13.0)
		_za = -_za;
	_zv += _za;*/
	_pos.z += _zv * dt;
	for (int i = 0; i < h_objParams.vnum; i++) {
		auto pos = p(i);
		Eigen::RowVector3d rpos;
		pos.z += _zv * dt;
		for (int j = 0; j < 3; j++) {
			h_objParams.vertices[i * 3 + j] = pos[j];
			_pos0[i * 3 + j] = pos[j];
			rpos[j] = pos[j];
		}
		rpos = rotateY(rpos, _w);
		_velocities.row(i) = (rpos - _nodes.row(i)) / dt;
		for (int j = 0; j < 3; j++)
			h_objParams.velocities[i * 3 + j] = _velocities.row(i)[j];
		_nodes.row(i) = rpos;
	}
	if (fabs(_pos.z) > 2.0)
		_zv = -_zv;
}
void Obstacle::computeNormal(void)
{
	int numFaces = _faces.rows();
	
	for (int i = 0; i < numFaces; i++) {
		auto f = _mesh->_faces[i];
		Vec3 a(_nodes.row(_faces.row(i)[0])[0], _nodes.row(_faces.row(i)[0])[1], _nodes.row(_faces.row(i)[0])[2]);
		Vec3 b(_nodes.row(_faces.row(i)[1])[0], _nodes.row(_faces.row(i)[1])[1], _nodes.row(_faces.row(i)[1])[2]);
		Vec3 c(_nodes.row(_faces.row(i)[2])[0], _nodes.row(_faces.row(i)[2])[1], _nodes.row(_faces.row(i)[2])[2]);
		Vec3 a2b = b - a;
		Vec3 a2c = c - a;
		Vec3 triNormal = Cross(a2b, a2c);
		triNormal = Normalize(triNormal);
		_normals(0, i) = triNormal.x;
		_normals(1, i) = triNormal.y;
		_normals(2, i) = triNormal.z;
	}
}
void Obstacle::draw(void)
{
	computeNormal();
	//drawWire();
	drawSurface();
	//_bvh->draw();
}
void Obstacle::drawWire(void)
{
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glColor3d(0, 0, 0);

	int numFaces = _faces.rows();
	for (int i = 0; i < numFaces; i++) {
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < 3; j++) {
			auto x = _nodes.row(_faces.row(i)[j])[0];
			auto y = _nodes.row(_faces.row(i)[j])[1];
			auto z = _nodes.row(_faces.row(i)[j])[2];
			glVertex3f(x, y, z);
		}
		glEnd();
	}

	glEnable(GL_LIGHTING);
	glShadeModel(GL_FLAT);
	glPopMatrix();
}
void Obstacle::drawSurface(void)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1); // turn on two-sided lighting.
	float purple[] = { 0.0f, 0.44705882352941176470588235294118f, 0.66666666666666666666666666666667f, 1.0f };
	float yellow[] = { 0.6f, 0.6f, 0.0f, 1.0f };
	float white[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	float black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white);
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
	glMaterialf(GL_FRONT, GL_SHININESS, 64);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, white); // back material
	glMaterialfv(GL_BACK, GL_SPECULAR, black); // no specular highlights

	int numFaces = _faces.rows();
	for (int i = 0; i < numFaces; i++) {
		auto f = _mesh->_faces[i];
		Vec3 a(_nodes.row(_faces.row(i)[0])[0], _nodes.row(_faces.row(i)[0])[1], _nodes.row(_faces.row(i)[0])[2]);
		Vec3 b(_nodes.row(_faces.row(i)[1])[0], _nodes.row(_faces.row(i)[1])[1], _nodes.row(_faces.row(i)[1])[2]);
		Vec3 c(_nodes.row(_faces.row(i)[2])[0], _nodes.row(_faces.row(i)[2])[1], _nodes.row(_faces.row(i)[2])[2]);
		glBegin(GL_TRIANGLES);
		glNormal3f(_normals(0, i), _normals(1, i), _normals(2, i));
		glVertex3f(a.x, a.y, a.z);
		glVertex3f(b.x, b.y, b.z);
		glVertex3f(c.x, c.y, c.z);
		glEnd();
	}
}