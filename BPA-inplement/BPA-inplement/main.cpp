#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;


int main()
{
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFile("bunny.ply", mesh);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	//pcl::io::savePCDFileASCII("bunny.pcd", *cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
	viewer->addPolygonMesh(mesh, "mesh");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

	//cout << cloud->size() << endl;

	cout << "OK!";
	cin.get();
	return 0;
}