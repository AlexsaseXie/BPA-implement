#include "Front.h"

void Front::insert_triangle_edges(Triangle &tri) {
	front.push_back(Edge(tri.points[0], tri.points[1], tri.points[2], tri.ball_center));
	front.push_back(Edge(tri.points[1], tri.points[2], tri.points[0], tri.ball_center));
	front.push_back(Edge(tri.points[2], tri.points[0], tri.points[1], tri.ball_center));
}

void Front::join_glue(EdgePtr eij, PointData & ek, TrianglePtr tri, bool is_used) {
	if (!is_used) {
		this->front.insert(pos ,Edge(tri->points[0], tri->points[1], tri->points[2], tri->ball_center));
		this->front.insert(pos ,Edge(tri->points[1], tri->points[2], tri->points[0], tri->ball_center));

		this->front.erase(pos);
		advance(pos, -2);
	}
	else {
		// on front(ek)
		int added = 0;

		for (int i = 0; i <= 1; i++) {
			Edge v1 = Edge(tri->points[i], tri->points[i+1], tri->points[(i+2) % 3], tri->ball_center);
			auto it = front.begin();
			for (it = front.begin(); it != front.end(); it++) {
				if ((it->vertices[0].second == v1.vertices[0].second && it->vertices[1].second == v1.vertices[1].second)
					|| (it->vertices[0].second == v1.vertices[1].second && it->vertices[1].second == v1.vertices[0].second))
					break;
			}
			if (it == front.end()) {
				//no coincident edge
				front.insert(pos, v1);
				added--;
			}
			else {
				//remove coincident edge
				front.erase(it);
			}
		}

		front.erase(pos);

		// Move iterator to the first added new edge
		if (added < 0)
			advance(pos, added);
		else
			// Reset the position
			pos = front.begin();
	}
}


