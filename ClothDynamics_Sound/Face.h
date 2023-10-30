#ifndef __FACE_H__
#define __FACE_H__

#pragma once
#include "Vertex.h"
#include <vector>

using namespace std;

class Face
{
public:
	int				_index;
	bool			_flag;
	Vec3			_normal;
	vector<Vertex*>	_vertices; // Triangle : num. vertex -> 3
	vector<Edge*>	_edges;
public:
	Face();
	Face(int index, Vertex *v0, Vertex *v1, Vertex *v2)
	{
		_flag = false;
		_index = index;
		_vertices.push_back(v0);
		_vertices.push_back(v1);
		_vertices.push_back(v2);
	}
	~Face();
public:
	int				getIndex(Vertex *v);
	int				hasVertex(Vertex *nv);
	int				hasEdge(Edge *ne);
	double			area(void);
	Vertex			*otherVertex(Edge *e);
	Vec3			getEdgeNormal(int i);
	inline Vertex	*v(int id) { return _vertices[id]; }
};

#endif
