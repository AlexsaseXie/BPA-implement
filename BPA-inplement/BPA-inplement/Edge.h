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
		set_id();
		active = true;
	}
	inline Edge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const pcl::PointNormal &_center) {
		set_id();
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
		id = _other.id;
	}
	inline ~Edge() {}


	inline Edge &operator=(const Edge &_other) { 
		this->vertices = _other.vertices; 
		this->opposite_vertex = _other.opposite_vertex;
		this->active = _other.active;
		this->ball_center = _other.ball_center;
		this->middle_point = _other.middle_point;
		this->id = _other.id;
		this->pivoting_r = _other.pivoting_r;
	}
	//bool operator<(const Edge &_other) const;
	inline bool operator==(const Edge &_other) const {
		if (this->vertices[0].second != _other.vertices[0].second)
			return false;
		if (this->vertices[1].second != _other.vertices[1].second)
			return false;
		if (this->opposite_vertex.second != _other.opposite_vertex.second)
			return false;
		return true;
	}
	//bool operator!=(const Edge &_other) const;


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
