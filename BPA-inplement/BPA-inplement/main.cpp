#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include "BPA.h"

using namespace std;
using namespace pcl;


int main()
{
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFile("bunny.ply", mesh);

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
	ne.setKSearch(6);

	//compute the features
	//cloud_normals->points.size() should have the same size as the input cloud->points.size
	ne.compute(*cloud_normals);

	pcl::PointCloud<PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);


	pcl::PolygonMesh mesh1;
	pcl::toPCLPointCloud2(*cloud_with_normals, mesh1.cloud);

	BPA api(0.0015);
	api.do_bpa(*cloud_with_normals, mesh1);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
	viewer->addPolygonMesh(mesh1, "mesh");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

	cout << "OK!";
	cin.get();
	return 0;
}