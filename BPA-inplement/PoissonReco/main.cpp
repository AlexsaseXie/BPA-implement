#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
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

	/*poission �ؽ��׶�*/
	//����poisson�ؽ�����
	Poisson<PointNormal> pn;

	pn.setConfidence(true); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
	pn.setDepth(8); //���������ȣ����2^d x 2^d x 2^d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
	pn.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
	pn.setManifold(true); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	pn.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	pn.setSamplesPerNode(3.0); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
	pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������


	//����poisson�ؽ���������
	pn.setInputCloud(cloud_with_normals);
	//poisson�ؽ���ʼ
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