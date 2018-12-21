#pragma once

#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <ostream>
#include "Util.h"

// Edge smart pointer
class Edge;
typedef Edge* EdgePtr;
typedef std::pair<pcl::PointNormal, int> PointData;

class Edge
{
public:
	Edge();
	Edge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const pcl::PointNormal &_center);
	Edge(const Edge &_other);
	~Edge();


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
