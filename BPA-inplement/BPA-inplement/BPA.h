#pragma once

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <vector>
#include "Edge.h"
#include "Front.h"
#include "Triangle.h"

using namespace std;
using namespace pcl;

class BPA {
public:
	inline BPA() {}
	inline ~BPA() {}

public:
	Front F;
	vector<bool> pt_used;

public:
	inline void init(pcl::PointCloud<pcl::PointNormal> &cloud) {
		pt_used = vector<bool>(cloud.size(), 0);
	}

	void do_bpa(pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PolygonMesh &mesh) {}
	EdgePtr get_active_edge();
	void mark_as_boundary(EdgePtr eij);
	PointData ball_pivot();

	void output_triangle(const PointData &a, const PointData &b, const PointData &c);

	pair<Triangle, bool> find_seed_triangle();
};