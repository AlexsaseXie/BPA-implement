#pragma once

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include <vector>
#include "Edge.h"
#include "Front.h"
#include "Triangle.h"

using namespace std;
using namespace pcl;

class BPA {
public:
	inline BPA(){}
	inline BPA(double r) : ball_radius(r) {}
	inline ~BPA() {}

public:
	Front F;
	vector<bool> pt_used;
	list<int> unused_index;

	double ball_radius = 5;
	pcl::PointCloud<pcl::PointNormal> *cloud;
	pcl::PolygonMesh *mesh;
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;

	vector<vector<int>> faces;

public:
	inline void init(pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PolygonMesh &mesh) {
		pt_used.clear();
		unused_index.clear();

		pt_used = vector<bool>(cloud.size(), 0);
		this->cloud = &cloud;
		this->mesh = &mesh;
		
		// insert unused_index
		for (int i = 0; i < cloud.size(); i++) {
			unused_index.push_back(i);
		}

		//initialize kdtree
		kdtree.setInputCloud(pcl::PointCloud<pcl::PointNormal>::Ptr(&cloud));

		faces.clear();
	}

	inline void set_used(int index) {
		auto it = find(unused_index.begin(), unused_index.end() ,index);
		if (it != unused_index.end()) unused_index.erase(it);
		pt_used[index] = true;
	}

	void do_bpa(pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PolygonMesh &mesh);
	EdgePtr get_active_edge();
	void mark_as_boundary(EdgePtr eij);
	TrianglePtr ball_pivot(EdgePtr eij);
	void output_triangle(const PointData &a, const PointData &b, const PointData &c);
	pair<Triangle, bool> find_seed_triangle();

public:
	vector<int> get_neighbors(const PointNormal &middle_point, double search_R);
	bool get_ball_center(const int _index0, const int _index1, const int _index2, Eigen::Vector3f &_center, Eigen::Vector3i &_sequence) const;
	bool is_empty(const std::vector<int> &_data, const int _index0, const int _index1, const int _index2, const Eigen::Vector3f &_ballCenter) const;
	std::pair<Eigen::Vector3f, double> get_circumscribed_circle(const Eigen::Vector3f &_p0, const Eigen::Vector3f &_p1, const Eigen::Vector3f &_p2) const;
};