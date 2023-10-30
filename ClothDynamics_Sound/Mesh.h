#ifndef __MESH_H__
#define __MESH_H__

#pragma once
#include <algorithm>
#include "Face.h"
#include "Edge.h"
#include "freeglut.h"

class Mesh
{
public:
	Vec3			_minBoundary;
	Vec3			_maxBoundary;
	Vec3			_minTexCoord;
	Vec3			_maxTexCoord;
	vector<Vertex*>	_vertices;
	vector<Face*>	_faces;
	vector<Edge*>	_stretchEdges; // edges
	vector<int>		_bendingEdges;
public:
	Mesh();
	Mesh(char *filename, bool resize = true)
	{
		loadObj(filename, resize);
	}
	~Mesh();
public:
	inline Face		*f(int i) { return _faces[i]; }
	inline Edge		*se(int i) { return _stretchEdges[i]; } // stretch edge
	inline int		be(int i) { return _bendingEdges[i]; } // bending edge
	inline Vertex	*v(int i) { return _vertices[i]; }
public:
	void	init(void);
	void	initEdgeGraph(void);
	void	loadObj(char *filename, bool resize = true);
	void	moveToCenter(double scale);
	void	computeNormal(void);
	void	buildAdjacency(void);
	void	offset(Vec3 offset);
	void	scale(double s);
	void	rotateX(double degree);
	void	computeVirtualTexCoord(void);
public:
	void	draw(Vec3 color);
	void	drawWire(void);
	void	drawSolid(Vec3 color);
	void	drawPoint(void);
};

#endif
