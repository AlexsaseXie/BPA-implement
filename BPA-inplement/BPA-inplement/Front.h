#pragma once

#include "Edge.h"
#include "Triangle.h"
#include <map>
#include <vector>
#include <list>

class Front
{
public:
	inline Front() { front = list<Edge>(); pos = front.begin(); }
	inline Front(int cloud_size) { front = list<Edge>(); pos = front.begin(); this->cloud_size = cloud_size; on_front_count = vector<int>(cloud_size, 0); }
	inline ~Front() {}

public:
	vector<int> on_front_count;
	int cloud_size;

public:
	inline EdgePtr get_first_active() {
		auto it = pos;
		bool first = true;

		if (front.empty())
			return NULL;

		while (1) {
			if (it == front.end()) {
				it = front.begin();
			}

			if (it == pos && !first)
				break;

			if (it->active == true) {
				pos = it;

				return &(*it);
			}
			
			first = false;
			it++;
		}

		pos = front.end();
		return NULL;
	}

	inline bool on_front(PointData &ek_data) {
		/*for (auto it = front.begin(); it != front.end(); it++) {
			if (it->vertices[0].second == ek_data.second || it->vertices[1].second == ek_data.second) {
				return true;
			}
		}
		return false;*/

		return on_front_count[ek_data.second] > 0;
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

	inline void clear() {
		front.clear();
		pos = front.begin();
		for (int i = 0; i < cloud_size; i++) {
			on_front_count[i] = 0;
		}
	}

	void insert_triangle_edges(Triangle & tri);
	void join_glue(EdgePtr eij, PointData & ek, TrianglePtr tri, bool is_used);

private:
	std::list<Edge> front;
	std::list<Edge>::iterator pos;
};
