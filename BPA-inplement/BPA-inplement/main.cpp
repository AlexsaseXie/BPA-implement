#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <string>
#include <set>
#include "BPA.h"

using namespace std;
using namespace pcl;



void merge(pcl::PolygonMesh& a, pcl::PolygonMesh &b) {
	set<string> a_set;
	for (auto &vs : a.polygons) {
		int i0 = vs.vertices[0];
		int i1 = vs.vertices[1];
		int i2 = vs.vertices[2];
		
		if (i0 > i1) {
			int t = i0;
			i0 = i1;
			i1 = t;
		}

		if (i1 > i2) {
			int t = i1;
			i1 = i2;
			i2 = t;
		}

		a_set.insert(to_string(i0) + "," + to_string(i1) + "," + to_string(i2));
	}

	for (auto &vs : b.polygons) {
		int i0 = vs.vertices[0];
		int i1 = vs.vertices[1];
		int i2 = vs.vertices[2];

		if (i0 > i1) {
			int t = i0;
			i0 = i1;
			i1 = t;
		}

		if (i1 > i2) {
			int t = i1;
			i1 = i2;
			i2 = t;
		}

		string target = to_string(i0) + "," + to_string(i1) + "," + to_string(i2);

		auto it = a_set.find(target);
		if (it == a_set.end()) {
			a.polygons.push_back(vs);
		}
	}
}

int main(int argc, char *argv[])
{
	string input_file = "bunny.ply";
	double ball_radius = 0.0012;
	double ball_radius2 = 0.002;
	if (argc > 1) 
		input_file = string(argv[1]);
	if (argc > 2)
		ball_radius = atof(argv[2]);
	if (argc > 3)
		ball_radius = atof(argv[3]);
	

	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFile(input_file, mesh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	//create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	//create an empty kdtree representation, and pass it to the normal estimation object
	//its content will be filled inside the object, based on the given input dataset(as no other search surface is given)
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	//Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	//use all neighbours in a sphere of radius 3cm
	ne.setKSearch(18);
	//ne.setRadiusSearch(0.003);

	//compute the features
	//cloud_normals->points.size() should have the same size as the input cloud->points.size
	ne.compute(*cloud_normals);

	pcl::PointCloud<PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);


	pcl::PolygonMesh mesh1;
	pcl::toPCLPointCloud2(*cloud_with_normals, mesh1.cloud);

	BPA api(ball_radius);
	api.do_bpa(*cloud_with_normals, mesh1);

	pcl::PolygonMesh mesh2;
	pcl::toPCLPointCloud2(*cloud_with_normals, mesh2.cloud);

	BPA api2(ball_radius2);
	api2.do_bpa(*cloud_with_normals, mesh2);

	cout << "Start Merge" << endl;

	merge(mesh1, mesh2);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
	viewer->addPolygonMesh(mesh1, "mesh");
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 5, 5, "normals");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

	cout << "OK!";
	cin.get();
	return 0;
}