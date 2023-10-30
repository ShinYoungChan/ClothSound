#include "Lib.h"
#include "Mesh.h"

Mesh::Mesh()
{
}


Mesh::~Mesh()
{
}

void Mesh::loadObj(char *filename, bool resize)
{
	FILE *fp;
	char header[256] = { 0 };
	double pos[3];
	int v_index[3];
	int index = 0;

	_minBoundary.Set(100000.0);
	_maxBoundary.Set(-100000.0);

	fopen_s(&fp, filename, "r");
	while (fscanf(fp, "%s %lf %lf %lf", header, &pos[0], &pos[1], &pos[2]) != EOF) {
		if (header[0] == 'v' && header[1] == NULL) {
			if (_minBoundary[0] > pos[0])	_minBoundary[0] = pos[0];
			if (_minBoundary[1] > pos[1])	_minBoundary[1] = pos[1];
			if (_minBoundary[2] > pos[2])	_minBoundary[2] = pos[2];
			if (_maxBoundary[0] < pos[0])	_maxBoundary[0] = pos[0];
			if (_maxBoundary[1] < pos[1])	_maxBoundary[1] = pos[1];
			if (_maxBoundary[2] < pos[2])	_maxBoundary[2] = pos[2];
			_vertices.push_back(new Vertex(index++, Vec3(pos[0], pos[1], pos[2])));
		} 
	}

	//_minTexCoord.Set(100000.0);	_maxTexCoord.Set(-100000.0);
	//index = 0;
	//fseek(fp, 0, SEEK_SET);
	//while (fscanf(fp, "%s %lf %lf %lf", header, &pos[0], &pos[1], &pos[2]) != EOF) {
	//	if (header[0] == 'v' && header[1] == 't') {
	//		pos[0] /= 255.0;
	//		pos[1] /= 255.0;
	//		_vertices[index]->_texCoord.Set(pos[0], pos[1], 0.0);
	//		_minTexCoord[0] = fmin(_minTexCoord[0], pos[0]);
	//		_minTexCoord[1] = fmin(_minTexCoord[1], pos[1]);
	//		_maxTexCoord[0] = fmax(_maxTexCoord[0], pos[0]);
	//		_maxTexCoord[1] = fmax(_maxTexCoord[1], pos[1]);
	//		index++;
	//	}
	//}
	computeVirtualTexCoord();

	index = 0;
	fseek(fp, 0, SEEK_SET);
	while (fscanf(fp, "%s %d %d %d", header, &v_index[0], &v_index[1], &v_index[2]) != EOF) {
		if (header[0] == 'f' && header[1] == NULL) {
			auto v0 = _vertices[v_index[0] - 1];
			auto v1 = _vertices[v_index[1] - 1];
			auto v2 = _vertices[v_index[2] - 1];
			_faces.push_back(new Face(index++, v0, v1, v2));
		}
	}
	if (resize) {
		moveToCenter(2.0);
	}
	printf("%d, %d\n", _vertices.size(), _faces.size());
	buildAdjacency();
	computeNormal();
	fclose(fp);
}

void Mesh::computeVirtualTexCoord(void)
{
	auto length = _maxBoundary - _minBoundary;
	auto minBound = _minBoundary;

	_minTexCoord.Set(100000.0);
	_maxTexCoord.Set(-100000.0);

	// texture-coord range : -1~1
	for (auto v : _vertices) {
		for (int i = 0; i < 2; i++) {
			v->_texCoord[i] = ((v->_pos[i] - minBound[i]) / length[i]) * 2.0 - 1.0;
		}
		_minTexCoord[0] = fmin(_minTexCoord[0], v->_texCoord[0]);
		_minTexCoord[1] = fmin(_minTexCoord[1], v->_texCoord[1]);
		_maxTexCoord[0] = fmax(_maxTexCoord[0], v->_texCoord[0]);
		_maxTexCoord[1] = fmax(_maxTexCoord[1], v->_texCoord[1]);
	}
}

void Mesh::scale(double s)
{
	auto cp = (_maxBoundary + _minBoundary) / 2.0;
	for (auto v : _vertices) {
		v->_pos = v->_pos + ((v->_pos - cp) * s);
	}
}

void Mesh::offset(Vec3 offset)
{
	for (auto v : _vertices) {
		v->_pos += offset;
	}
}

void Mesh::buildAdjacency(void)
{
	for (auto v : _vertices) {
		v->_nbFaces.clear();
		v->_nbVertices.clear();
	}

	// v-f
	for (auto f : _faces) {
		for (int j = 0; j < 3; j++) {
			f->_vertices[j]->_nbFaces.push_back(f);
		}
	}

	// v-v
	for (auto v : _vertices) {
		for (auto nf : v->_nbFaces) {
			auto pivot = nf->getIndex(v); // 0 : 1,2, 1 : 2,0, 2: 0,1
			int other_id0 = (pivot + 1) % 3;
			int other_id1 = (pivot + 2) % 3;
			if (!v->hasNbVertex(nf->_vertices[other_id0])) {
				v->_nbVertices.push_back(nf->_vertices[other_id0]);
			}
			if (!v->hasNbVertex(nf->_vertices[other_id1])) {
				v->_nbVertices.push_back(nf->_vertices[other_id1]);
			}
		}
	}

	initEdgeGraph();
}

void Mesh::init(void)
{
	for (auto v : _vertices) {
		v->_flag = false;
	}

	for (auto f : _faces) {
		f->_flag = false;
	}
}

void Mesh::initEdgeGraph(void)
{
	int index = 0;

	// stretch edge graph
	for (auto v : _vertices) {
		for (auto nv : v->_nbVertices) {
			if (!nv->_flag) {
				_stretchEdges.push_back(new Edge(index++, v, nv));
			}
		}
		v->_flag = true;
	}
	init();

	// e-f
	for (auto e : _stretchEdges) {
		auto v = e->v(0);
		for (auto nf : v->_nbFaces) {
			if (nf->hasVertex(e->v(0)) && nf->hasVertex(e->v(1))) {
				if (!nf->hasEdge(e)) {
					nf->_edges.push_back(e);
				}
				if (!e->hasFace(nf)) {
					e->_nbFaces.push_back(nf);
				}
			}
		}
	}

	// bending edge graph
	index = 0;
	_bendingEdges.clear();	
	for (auto e : _stretchEdges) {
		if (e->_nbFaces.size() == 2) { // num. of neighbor faces : 2
			_bendingEdges.push_back(e->_index);
		}
	}
}

void Mesh::computeNormal(void)
{
	//// f-normal
	//for (auto f : _faces) {
	//	auto a = f->_vertices[0]->_pos;
	//	auto b = f->_vertices[1]->_pos;
	//	auto c = f->_vertices[2]->_pos;
	//	auto normal = Cross(a - b,a - c);
	//	normal.SetNormalizedVector();
	//	f->_normal = normal;
	//}

	//// v-normal
	//for (auto v : _vertices) {
	//	v->_normal.Set(0.0);
	//	for (auto nf : v->_nbFaces) {
	//		v->_normal += nf->_normal;
	//	}
	//	v->_normal /= v->_nbFaces.size();
	//}
	for (auto v : _vertices)
		v->_normal.Set(0.0);

	for (auto f : _faces) {
		auto a = f->_vertices[0]->_pos;
		auto b = f->_vertices[1]->_pos;
		auto c = f->_vertices[2]->_pos;
		auto normal = Cross(a - b, a - c);
		normal.SetNormalizedVector();
		f->_normal = normal;
		f->_vertices[0]->_normal += normal * AngleBetweenVectors(a - b, a - c);
		f->_vertices[1]->_normal += normal * AngleBetweenVectors(b - a, b - c);
		f->_vertices[2]->_normal += normal * AngleBetweenVectors(c - a, c - b);
	}

	for (auto v : _vertices)
		v->_normal.SetNormalizedVector();
}

void Mesh::moveToCenter(double scale)
{
	double max_length = fmax(fmax(_maxBoundary[0] - _minBoundary[0], _maxBoundary[1] - _minBoundary[1]), _maxBoundary[2] - _minBoundary[2]);
	auto center = (_maxBoundary + _minBoundary) / 2.0;
	Vec3 new_center(0.0, 0.0, 0.0);

	_minBoundary.Set(100000.0);
	_maxBoundary.Set(-100000.0);

	for (auto v : _vertices) {
		auto pos = v->_pos;
		auto grad = pos - center;
		grad /= max_length;
		grad *= scale;
		pos = new_center;
		pos += grad;
		v->_pos = pos;

		if (_minBoundary[0] > pos[0])	_minBoundary[0] = pos[0];
		if (_minBoundary[1] > pos[1])	_minBoundary[1] = pos[1];
		if (_minBoundary[2] > pos[2])	_minBoundary[2] = pos[2];
		if (_maxBoundary[0] < pos[0])	_maxBoundary[0] = pos[0];
		if (_maxBoundary[1] < pos[1])	_maxBoundary[1] = pos[1];
		if (_maxBoundary[2] < pos[2])	_maxBoundary[2] = pos[2];
	}
}

void Mesh::rotateX(double degree)
{
	for (auto v : _vertices) {
		Vec3 pos(v->_pos.x, v->_pos.y, v->_pos.z);
		auto new_pos = RotateX(pos, degree);
		for (int j = 0; j < 3; j++) {
			v->_pos[j] = new_pos[j];
		}
	}
	computeNormal();
}

void Mesh::drawPoint(void)
{
	glDisable(GL_LIGHTING);
	glPointSize(3.0f);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for (auto v : _vertices) {
		glVertex3f(v->x(), v->y(), v->z());
	}
	glEnd();
	glPointSize(1.0f);
	glEnable(GL_LIGHTING);
}

void Mesh::draw(Vec3 color)
{
	drawWire();
	//drawSolid(color);
}

void Mesh::drawSolid(Vec3 color)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1); // Turn on two-sided lighting.
	float mesh_front[] = { (float)color.x, (float)color.y, (float)color.z, 1.0f };
	float mesh_back[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float white[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	float black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mesh_front);
	//glMaterialfv(GL_FRONT, GL_SPECULAR, white);
	glMaterialf(GL_FRONT, GL_SHININESS, 64);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, mesh_back);  // Back material
	glMaterialfv(GL_BACK, GL_SPECULAR, black);  // No specular highlights

	for (auto f : _faces) {
		glBegin(GL_POLYGON);
		glNormal3f(f->_normal.x, f->_normal.y, f->_normal.z);
		for (int j = 0; j < 3; j++) {
			glVertex3f(f->_vertices[j]->x(), f->_vertices[j]->y(), f->_vertices[j]->z());
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
}

void Mesh::drawWire(void)
{
	glDisable(GL_LIGHTING);
	glColor3f(0.0f, 0.0f, 0.0f);
	for (auto f : _faces) {
		glBegin(GL_LINES);
		for (int j = 0; j < 3; j++) {
			int s = j % 3; // 0,1,2
			int e = (j + 1) % 3; // 0,1,2
			glVertex3f(f->_vertices[s]->x(), f->_vertices[s]->y(), f->_vertices[s]->z());
			glVertex3f(f->_vertices[e]->x(), f->_vertices[e]->y(), f->_vertices[e]->z());
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
}