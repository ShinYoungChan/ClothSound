#include "BVHHost2.h"

BVHHost2::BVHHost2(ObjectParams& objParams, const NbFaceBuffer& nbfaces) {
	_root = new BVHNodeHost2();
	for (int i = 0; i < objParams.fnum; i++) {
		int i0 = objParams.faces[i * 3 + 0] * 3;
		int i1 = objParams.faces[i * 3 + 1] * 3;
		int i2 = objParams.faces[i * 3 + 2] * 3;

		Vec3 v0(objParams.vertices[i0 + 0], objParams.vertices[i0 + 1], objParams.vertices[i0 + 2]);
		Vec3 v1(objParams.vertices[i1 + 0], objParams.vertices[i1 + 1], objParams.vertices[i1 + 2]);
		Vec3 v2(objParams.vertices[i2 + 0], objParams.vertices[i2 + 1], objParams.vertices[i2 + 2]);
		_root->Min(v0 + v1 + v2);
		_root->Max(v0 + v1 + v2);
		_root->_faces.push_back(i);
	}
	vector<BVHNodeHost2*> queue;
	queue.push_back(_root);
	while (queue.size()) {
		auto node = queue[0];
		queue.erase(queue.begin());
		if (node->_faces.size() == 0)
			continue;
		if (node->_faces.size() == 1) {
			int inf = node->_faces[0];
			int i0 = objParams.faces[inf * 3 + 0];
			int i1 = objParams.faces[inf * 3 + 1];
			int i2 = objParams.faces[inf * 3 + 2];
			for (int i = nbfaces.index[i0]; i < nbfaces.index[i0 + 1]; i++) {
				if (nbfaces.array[i] < inf)
					node->RTriVertex(0, 0);
			}
			for (int i = nbfaces.index[i1]; i < nbfaces.index[i1 + 1]; i++) {
				if (nbfaces.array[i] < inf)
					node->RTriVertex(1, 0);
			}
			for (int i = nbfaces.index[i2]; i < nbfaces.index[i2 + 1]; i++) {
				if (nbfaces.array[i] < inf)
					node->RTriVertex(2, 0);
			}
			bool check = false;
			for (int i = nbfaces.index[i0]; i < nbfaces.index[i0 + 1]; i++) {
				if (nbfaces.array[i] == inf)
					continue;
				for (int j = nbfaces.index[i1]; j < nbfaces.index[i1 + 1]; j++) {
					if (nbfaces.array[i] == nbfaces.array[j]) {
						if(nbfaces.array[i] < inf)
							node->RTriEdge(0, 0);
						check = true;
						break;
					}
				}
				if (check)
					break;
			}
			check = false;
			for (int i = nbfaces.index[i1]; i < nbfaces.index[i1 + 1]; i++) {
				if (nbfaces.array[i] == inf)
					continue;
				for (int j = nbfaces.index[i2]; j < nbfaces.index[i2 + 1]; j++) {
					if (nbfaces.array[i] == nbfaces.array[j]) {
						if (nbfaces.array[i] < inf)
							node->RTriEdge(1, 0);
						check = true;
						break;
					}
				}
				if (check)
					break;
			}
			check = false;
			for (int i = nbfaces.index[i2]; i < nbfaces.index[i2 + 1]; i++) {
				if (nbfaces.array[i] == inf)
					continue;
				for (int j = nbfaces.index[i0]; j < nbfaces.index[i0 + 1]; j++) {
					if (nbfaces.array[i] == nbfaces.array[j]) {
						if (nbfaces.array[i] < inf)
							node->RTriEdge(2, 0);
						check = true;
						break;
					}
				}
				if (check)
					break;
			}
		}
		else if (node->_faces.size() == 2) {
			node->_childs[0] = new BVHNodeHost2(node->_level + 1);
			node->_childs[1] = new BVHNodeHost2(node->_level + 1);
			node->_childs[0]->_faces.push_back(node->_faces[0]);
			node->_childs[1]->_faces.push_back(node->_faces[1]);
			queue.push_back(node->_childs[0]);
			queue.push_back(node->_childs[1]);
		}
		else {
			double maxDist = 0.0;
			int divAxis = 0;
			for (int i = 0; i < 3; i++) {
				double dist = node->_max[i] - node->_min[i];
				if (maxDist < dist) {
					maxDist = dist;
					divAxis = i;
				}
			}
			double divPos = 0.0;
			for (auto i : node->_faces)
				divPos +=
				objParams.vertices[objParams.faces[i * 3 + 0] * 3 + divAxis] +
				objParams.vertices[objParams.faces[i * 3 + 1] * 3 + divAxis] +
				objParams.vertices[objParams.faces[i * 3 + 2] * 3 + divAxis];
			divPos /= (double)node->_faces.size();

			node->_childs[0] = new BVHNodeHost2(node->_level + 1);
			node->_childs[1] = new BVHNodeHost2(node->_level + 1);

			for (auto i : node->_faces) {
				int i0 = objParams.faces[i * 3 + 0] * 3;
				int i1 = objParams.faces[i * 3 + 1] * 3;
				int i2 = objParams.faces[i * 3 + 2] * 3;
				Vec3 v0(objParams.vertices[i0 + 0], objParams.vertices[i0 + 1], objParams.vertices[i0 + 2]);
				Vec3 v1(objParams.vertices[i1 + 0], objParams.vertices[i1 + 1], objParams.vertices[i1 + 2]);
				Vec3 v2(objParams.vertices[i2 + 0], objParams.vertices[i2 + 1], objParams.vertices[i2 + 2]);
				double center = v0[divAxis] + v1[divAxis] + v2[divAxis];
				if (center < divPos) {
					node->_childs[0]->_faces.push_back(i);
					node->_childs[0]->Min(v0 + v1 + v2);
					node->_childs[0]->Max(v0 + v1 + v2);
				}
				else {
					node->_childs[1]->_faces.push_back(i);
					node->_childs[1]->Min(v0 + v1 + v2);
					node->_childs[1]->Max(v0 + v1 + v2);
				}
			}
			node->_faces.clear();
			queue.push_back(node->_childs[0]);
			queue.push_back(node->_childs[1]);
		}
	}
}

void BVHHost2::clear(void) {
	delete _root;
	_root = nullptr;
}
void BVHHost2::draw(void) {
	if (!_root)
		return;
	vector<BVHNodeHost2*> queue;
	queue.push_back(_root);
	while (queue.size()) {
		auto node = queue[0];
		queue.erase(queue.begin());
		if (node->_childs[0]) {
			queue.push_back(node->_childs[0]);
			queue.push_back(node->_childs[1]);
			//continue;
		}
		node->draw();
	}
}
void BVHHost2::refit(const vector<int>& faces, const vector<double>& vertices, const vector<double>& velocities, double thickness, double dt, bool isCCD) {
	if (!_root)
		return;
	_root->refit(faces, vertices, velocities, thickness * 0.5, dt, isCCD);
}
void BVHHost2::getNearestFaces(const Vec3& p) {
	_nfaces.clear();
	if (!_root)
		return;

	vector<BVHNodeHost2*> queue;
	queue.push_back(_root);
	while (queue.size()) {
		auto node = queue[0];
		queue.erase(queue.begin());

		if (node->intersect(p)) {
			if (node->_childs[0]) {
				queue.push_back(node->_childs[0]);
				queue.push_back(node->_childs[1]);
				continue;
			}
			_nfaces.insert(_nfaces.end(), node->_faces.begin(), node->_faces.end());
		}
	}
}
void BVHHost2::getNearestFaces(const Vec3& pa, const Vec3& pb, const Vec3& pc) {
	_nfaces.clear();
	if (!_root)
		return;

	vector<BVHNodeHost2*> queue;
	queue.push_back(_root);
	while (queue.size()) {
		auto node = queue[0];
		queue.erase(queue.begin());

		if (node->intersect(pa, pb, pc)) {
			if (node->_childs[0]) {
				queue.push_back(node->_childs[0]);
				queue.push_back(node->_childs[1]);
				continue;
			}
			_nfaces.insert(_nfaces.end(), node->_faces.begin(), node->_faces.end());
		}
	}
}
void BVHHost2::getNearestFaces(const Vec3& p0, const Vec3& p1) {
	_nfaces.clear();
	if (!_root)
		return;

	vector<BVHNodeHost2*> queue;
	queue.push_back(_root);
	while (queue.size()) {
		auto node = queue[0];
		queue.erase(queue.begin());

		if (node->intersect(p0, p1)) {
			if (node->_childs[0]) {
				queue.push_back(node->_childs[0]);
				queue.push_back(node->_childs[1]);
				continue;
			}
			_nfaces.insert(_nfaces.end(), node->_faces.begin(), node->_faces.end());
		}
	}
}
void BVHHost2::getNearestFaces(const Vec3& pa0, const Vec3& pb0, const Vec3& pc0, const Vec3& pa1, const Vec3& pb1, const Vec3& pc1) {
	_nfaces.clear();
	if (!_root)
		return;

	vector<BVHNodeHost2*> queue;
	queue.push_back(_root);
	while (queue.size()) {
		auto node = queue[0];
		queue.erase(queue.begin());

		if (node->intersect(pa0, pb0, pc0, pa1, pb1, pc1)) {
			if (node->_childs[0]) {
				queue.push_back(node->_childs[0]);
				queue.push_back(node->_childs[1]);
				continue;
			}
			_nfaces.insert(_nfaces.end(), node->_faces.begin(), node->_faces.end());
		}
	}
}