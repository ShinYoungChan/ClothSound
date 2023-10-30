#ifndef __VERTEX_H__
#define __VERTEX_H__

#pragma once
#include <vector>
#include <set>
#include "Vec3.h"

using namespace std;

class Face;
class Edge;
class Vertex
{
public:
	int				_index;
	int				_level; // n-ring
	bool			_flag;
	bool			_fixed;
	Vec3			_pos; 
	Vec3			_normal;
	Vec3			_texCoord;			// texture coordinate
	vector<Face*>	_nbFaces;			// neighbor face
	vector<Vertex*>	_nbVertices;		// neighbor vertex
	vector<Edge*>	_nbEdges;			// neighbor edge
	set<int>		_auxiliarySpring;	// 2-ring neighbor vertices
public:
	Vertex();
	Vertex(int index, Vec3 pos)
	{
		_level = 0;
		_flag = false;
		_index = index;
		_pos = pos;
		_fixed = false;
	}
	~Vertex();
public:
	inline double x(void) { return _pos.x; }
	inline double y(void) { return _pos.y; }
	inline double z(void) { return _pos.z; }
public:
	bool	hasNbVertex(Vertex *v);
};

#endif