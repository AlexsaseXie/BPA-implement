#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <string>
#include <pcl/surface/poisson.h>



using namespace std;
using namespace pcl;


int main(int argc, char *argv[])
{
	string input_file = "horse.ply";

	if (argc > 1) {
		input_file = string(argv[1]);
	}


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
	ne.setKSearch(15);

	//compute the features
	//cloud_normals->points.size() should have the same size as the input cloud->points.size
	ne.compute(*cloud_normals);

	pcl::PointCloud<PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);


	pcl::PolygonMesh mesh1;

	/*poission 重建阶段*/
	//创建poisson重建对象
	Poisson<PointNormal> pn;

	pn.setConfidence(true); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	pn.setManifold(true); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度


	//输入poisson重建点云数据
	pn.setInputCloud(cloud_with_normals);
	//poisson重建开始
	pn.reconstruct(mesh1);


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