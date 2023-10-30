#include "Face.h"
#include "Edge.h"

Face::Face()
{
}


Face::~Face()
{
}

int	Face::getIndex(Vertex *v)
{
	for (int i = 0; i < 3; i++) {
		if (_vertices[i] == v) {
			return i;
		}
	}
	return -1;
}

int Face::hasVertex(Vertex *nv)
{
	for (auto v : _vertices) {
		if (v == nv) {
			return 1;
		}
	}
	return 0;
}

double Face::area(void)
{
	auto n = Cross(_vertices[1]->_pos - _vertices[0]->_pos,_vertices[2]->_pos - _vertices[0]->_pos);
	return Length(n) * 0.5;
}

int Face::hasEdge(Edge *ne)
{
	for (auto e : _edges) {
		if (e == ne) {
			return 1;
		}
	}
	return 0;
}

Vec3 Face::getEdgeNormal(int i)
{
	Vec3 norm;
	for (auto f : _vertices[i]->_nbFaces) {
		if (this != f && f->getIndex(_vertices[(i + 1) % 3]) != -1) {
			norm = f->_normal + _normal;
			norm.SetNormalizedVector();
			return norm;
		}
	}
	return _normal;
}

Vertex *Face::otherVertex(Edge *e)
{
	for (auto v : _vertices) {
		if (e->v(0) != v && e->v(1) != v) {
			return v;
		}
	}
	return nullptr;
}

