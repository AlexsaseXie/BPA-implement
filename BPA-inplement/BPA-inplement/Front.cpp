#include "Front.h"

void Front::insert_triangle_edges(Triangle &tri) {
	bool flag = false;
	if (front.size() == 0)
		flag = true;

	front.push_back(Edge(tri.points[0], tri.points[1], tri.points[2], tri.ball_center));
	front.push_back(Edge(tri.points[1], tri.points[2], tri.points[0], tri.ball_center));
	front.push_back(Edge(tri.points[2], tri.points[0], tri.points[1], tri.ball_center));

	if (flag)
		pos = front.begin();

	on_front_count[tri.points[0].second] += 2;
	on_front_count[tri.points[1].second] += 2;
	on_front_count[tri.points[2].second] += 2;
}

void Front::join_glue(EdgePtr eij, PointData & ek, TrianglePtr tri, bool is_used) {
	if (!is_used) {
		this->front.insert(pos ,Edge(tri->points[0], tri->points[1], tri->points[2], tri->ball_center));
		this->front.insert(pos ,Edge(tri->points[1], tri->points[2], tri->points[0], tri->ball_center));

		this->front.erase(pos);
		advance(pos, -2);

		on_front_count[tri->points[1].second] += 2;
	}
	else {
		// on front(ek)
		//cout << "Glue: " << "Tri :" << tri->points[0].second << "," << tri->points[1].second << "," << tri->points[2].second << endl;

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
				on_front_count[v1.vertices[0].second] += 1;
				on_front_count[v1.vertices[1].second] += 1;
				//no coincident edge
				front.insert(pos, v1);
				added--;
			}
			else {
				on_front_count[it->vertices[0].second] -= 1;
				on_front_count[it->vertices[1].second] -= 1;

				//remove coincident edge
				//front.erase(it);

				it->active = false;
			}
		}

		on_front_count[pos->vertices[0].second] -= 1;
		on_front_count[pos->vertices[1].second] -= 1;

		front.erase(pos);

		// Move iterator to the first added new edge
		if (added < 0)
			advance(pos, added);
		else
			// Reset the position
			pos = front.begin();
	}
}


