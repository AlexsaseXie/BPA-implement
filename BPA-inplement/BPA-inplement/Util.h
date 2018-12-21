#pragma once

#include<pcl/common/common.h>

using namespace pcl;

class Util {
public:
	inline static PointNormal make_pt_normal(const float x, const float y, const float z, const float nx = 0, const float ny = 0, const float nz = 0, const float curvature = 0) {
		PointNormal pt;
		pt.x = x;
		pt.y = y;
		pt.z = z;
		pt.normal_x = nx;
		pt.normal_y = ny;
		pt.normal_z = nz;
	}


};