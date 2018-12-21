#pragma once

#include<pcl/common/common.h>
#include<Eigen/src/Core/Matrix.h>

#define COMPARISON_EPSILON	1e-10

using namespace pcl;

// Sign function
template<typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

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

		return pt;
	}

	inline static PointNormal make_pt_normal(Eigen::Vector3f &c, const float nx = 0, const float ny = 0, const float nz = 0, const float curvature = 0) {
		PointNormal pt;
		pt.x = c.x();
		pt.y = c.y();
		pt.z = c.z();
		pt.normal_x = nx;
		pt.normal_y = ny;
		pt.normal_z = nz;

		return pt;
	}

	inline static bool is_oriented(const Eigen::Vector3f &_normal, const Eigen::Vector3f &_normal0, const Eigen::Vector3f &_normal1, const Eigen::Vector3f &_normal2) {
		int count = 0;
		count = _normal0.dot(_normal) < 0 ? count + 1 : count;
		count = _normal1.dot(_normal) < 0 ? count + 1 : count;
		count = _normal2.dot(_normal) < 0 ? count + 1 : count;

		return count <= 1;
	}
};