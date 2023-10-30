#include "Edge.h"


Edge::Edge()
{
}


Edge::~Edge()
{
}

double Edge::length(void)
{
	return Length(_vertices[0]->_pos - _vertices[1]->_pos);
}

int Edge::hasFace(Face *nf)
{
	for (auto f : _nbFaces) {
		if (f == nf) {
			return 1;
		}
	}
	return 0;
}