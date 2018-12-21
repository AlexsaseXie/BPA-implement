#pragma once

#include <pcl/common/common.h>
#include <vector>
#include "Edge.h"

using namespace std;
using namespace pcl;

class Triangle;
typedef Triangle* TrianglePtr;

class Triangle {
public:
	inline Triangle(){}
	inline Triangle(PointData &v1, PointData &v2, PointData &v3) {
		points = vector<PointData>(3);

		points[0] = (v1);
		points[1] = (v2);
		points[2] = (v3);
	}
	inline Triangle(PointData &v1, PointData &v2, PointData &v3, PointNormal &_center, double _r) {
		points = vector<PointData>(3);

		points[0] = (v1);
		points[1] = (v2);
		points[2] = (v3);

		ball_center = _center;
		ball_radius = _r;
	}

	inline ~Triangle() {}

public:
	vector<PointData> points;
	PointNormal ball_center;
	double ball_radius;
};