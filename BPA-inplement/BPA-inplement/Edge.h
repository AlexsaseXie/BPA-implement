#pragma once

#include <pcl/common/common.h>
#include <vector>
#include <ostream>
#include "Util.h"

// Edge smart pointer
class Edge;
typedef Edge* EdgePtr;
typedef std::pair<pcl::PointNormal * , int> PointData;

class Edge
{
public:
	inline Edge() {
		active = true;
	}
	inline Edge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const pcl::PointNormal &_center) {
		vertices.push_back(_v0);
		vertices.push_back(_v1);

		opposite_vertex = _opposite;
		ball_center = _center;
		PointNormal *ei = vertices[0].first;
		PointNormal *ej = vertices[1].first;
		middle_point.x = (ei->x + ej->x) / 2.0;
		middle_point.y = (ei->y + ej->y) / 2.0;
		middle_point.z = (ei->z + ej->z) / 2.0;

		active = true;
	}
	inline Edge(const Edge &_other) {
		vertices = _other.vertices;
		opposite_vertex = _other.opposite_vertex;
		ball_center = _other.ball_center;
		middle_point = _other.middle_point;
		active = _other.active;
	}
	inline ~Edge() {}


	Edge &operator=(const Edge &_other);
	bool operator<(const Edge &_other) const;
	bool operator==(const Edge &_other) const;
	bool operator!=(const Edge &_other) const;


private:
	inline void set_id()
	{
		static long currentId = 0;
		id = currentId++;
	}
	long id;

public:
	std::vector<PointData> vertices;
	PointData opposite_vertex;
	pcl::PointNormal ball_center;
	pcl::PointNormal middle_point;
	double pivoting_r;
	bool active;
};
