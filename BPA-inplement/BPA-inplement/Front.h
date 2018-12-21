#pragma once

//#include "Triangle.h"
//#include "Pivoter.h"
#include "Edge.h"
#include <map>
#include <list>

class Front
{
public:
	inline Front() {}
	inline ~Front() {}

public:
	inline EdgePtr get_first_active() {
		auto it = front.begin();
		while (it != front.end()) {
			if (it->active == true) {
				return &(*it);
			}
			it++;
		}
		return NULL;
	}

	inline bool on_front(PointData &ek_data) {
		for (auto it = front.begin(); it != front.end(); it++) {
			if (it->vertices[0].second == ek_data.second || it->vertices[1].second == ek_data.second) {
				return true;
			}
		}
		return false;
	}

	inline bool has(Edge &m) {
		auto it = find(front.begin(), front.end(), m);

		if (it != front.end()) {
			return true;
		}
		else {
			return false;
		}
	}

	inline bool has(const PointData &v1, const PointData &v2) {
		for (auto it = front.begin(); it != front.end(); it++) {
			if (it->vertices[0] == v1 && it->vertices[1] == v2) {
				return true;
			}
		}
		return false;
	}


	void insert_triangle_edges(Triangle & tri);
	void join(Edge & eij, PointData & ek);
	void glue(PointData & ei, PointData & ek);

private:

	std::list<Edge> front;
	std::list<Edge>::iterator pos;
	//std::map<int, std::map<EdgePtr, std::list<EdgePtr>::iterator> > points;
};
