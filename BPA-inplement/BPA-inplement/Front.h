#pragma once

#include "Edge.h"
#include "Triangle.h"
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
				pos = it;

				return &(*it);
			}
			it++;
		}

		pos = front.end();
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
			if (it->vertices[0].second == v1.second && it->vertices[1].second == v2.second) {
				return true;
			}
		}
		return false;
	}


	void insert_triangle_edges(Triangle & tri);
	void join_glue(EdgePtr eij, PointData & ek, TrianglePtr tri, bool is_used);

private:

	std::list<Edge> front;
	std::list<Edge>::iterator pos;
};
