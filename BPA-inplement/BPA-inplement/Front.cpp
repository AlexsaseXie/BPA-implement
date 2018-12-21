#include "Front.h"

void Front::insert_triangle_edges(Triangle &tri) {
	front.push_back(Edge(tri.points[0], tri.points[1], tri.points[2], tri.ball_center));
	front.push_back(Edge(tri.points[1], tri.points[2], tri.points[0], tri.ball_center));
	front.push_back(Edge(tri.points[2], tri.points[0], tri.points[1], tri.ball_center));
}


