//
// Created by ubuntu on 2021/3/10.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include "ProcessPointClouds.h"
#include "utils/Httplib.h"
#include "SM45.h"

using namespace std;

#define testSM 0
#define testPost 0
#define cloud2mat 0
#define horizonplanesegment 0
#define edgeFromgood 0
#define edgeFromraw 1


#if testSM
//test sm
int main()
{
    shared_ptr<SM45> sm_ptr = make_shared<SM45>();
    std::string port  = "/dev/ttyUSB0";
    sm_ptr->init(port, 115200);
    sm_ptr->start();

}
#endif

#if testPost
//test post
int main()
{
    std::shared_ptr<Httplib> client = std::make_shared<Httplib>();
    string im1 = "/home/ubuntu/CLionProjects/LiDAR-Point-Cloud-Compression/build/out.png";
    string im = "/home/ubuntu/Pictures/tsdf.png";
    string page = "http://localhost:9080/my";
    float detaX = 0.2;
    float detaY = 0.3;
    float minX = 8;
    float minY = 10;
//    client->Send(im, "82.156.16.242", 9081, "/my", detaX, detaY, minX, minY);
    client->Send(im, "localhost", 9081, "/my", detaX, detaY, minX, minY);
}
#endif

#if cloud2mat
//test cloud2mat
int main()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("look"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd = "/home/ubuntu/lidar/2/combine.pcd";
    std::string pcd1 = "/home/ubuntu/lidar/6pos2048.pcd";
    std::string pcd2 = "/home/ubuntu/lidar/poshe1/6pos2090.pcd";
    pcl::io::loadPCDFile(pcd2, *cloud);
    std::shared_ptr<ProcessPointClouds> ppc = std::make_shared<ProcessPointClouds>();

    cloud = ppc->DownSampleCloud(cloud, 0.6);
    std::vector<float> params = ppc->GetFov(cloud);
    float xmin = params[0];
    float ymin = params[1];
    float xlen = params[2];
    float ylen = params[3];
    std::cout<<"xmin: "<<xmin<<std::endl;
    std::cout<<"ymin: "<<ymin<<std::endl;
    std::cout<<"xlen: "<<xlen<<std::endl;
    std::cout<<"ylen: "<<ylen<<std::endl;
    float detaX = 0.3, detaY = 0.3;
    cv::Mat img, out;
    ppc->Cloud2Mat(cloud, img, detaX, detaY, xmin, ymin, xlen, ylen);
    cv::imwrite("/home/ubuntu/lidar/out2.png", img);

    out = cv::imread("/home/ubuntu/lidar/out2.png", -1);
    ppc->Mat2Cloud(out, detaX, detaY,xmin, ymin, cloudout);

    viewer->addPointCloud(cloud, "origin");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloudout, 0,255,0);
    viewer->addPointCloud(cloudout, g, "out");
    viewer->spin();
}
#endif

//nuscenes(x, y, z, intensity, ring index)
void readKittiBin2Pcd(std::string& in_file, std::string& out_file)
{
    // load point cloud file.
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good())
    {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

    int i;
    for(i=0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZ point;
        float ins;
        input.read((char *)&point.x, 3*sizeof(float));
        // input.read((char *)&point.intensity, sizeof(float));
        input.read((char *)&ins, sizeof(float));
        input.read((char *)&ins, sizeof(float));

        points->push_back(point);
    }
    input.close();
    std::cout << "Read KITTI point cloud with " << i << " points, writing to " << out_file << std::endl;

    pcl::PCDWriter writer; // save pcd file format
    // pcl::PLYWriter writer; // save ply file format
    writer.write<pcl::PointXYZ> (out_file, *points, false);
}

#if horizonplanesegment
//test horizonplanesegment
int main()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer!"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<PointT>);
    std::string bin = "/home/ubuntu/PycharmProjects/CenterPoint/download/v1.0-mini/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151611896734.pcd.bin";
    std::string pcd = "/home/ubuntu/1.pcd";
    readKittiBin2Pcd(bin, pcd);

    std::string pcd1 = "/home/ubuntu/lidar/6pos2048.pcd";
    pcl::io::loadPCDFile(pcd1, *cloud);


    //ProcessPointClouds *ppc = new ProcessPointClouds;
    std::shared_ptr<ProcessPointClouds> ppc_ptr = std::make_shared<ProcessPointClouds>();

    std::cout<<"cloud size: "<<cloud->size()<<std::endl;
    cloud = ppc_ptr->DownSampleCloud(cloud, 0.5);
    std::cout<<"downsample cloud size: "<<cloud->size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> b(cloud, 255, 0, 0);
    viewer->addPointCloud(cloud, b, "boundry");

    // show normal
//    pcl::PointCloud<pcl::Normal>::Ptr nn = ppc_ptr->GetNormals(cloud);
//    viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud, nn, 2, 0.6); //每2个点显示一个及每个法线的长度0.3

    //show segmentplaneHorizon
    PtCdPtr plane(new pcl::PointCloud<PointT>);
    PtCdPtr others(new pcl::PointCloud<PointT>);
    std::pair<PtCdPtr, PtCdPtr> result = ppc_ptr->SegmentPlaneHorizon(cloud, 50, 0.2);

    std::pair<PtCdPtr, PtCdPtr> result1 = ppc_ptr->SegmentPlaneHorizon(result.second, 50, 0.2);
    plane = result.first;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(plane, 0, 255, 0);
    viewer->addPointCloud(plane, g, "horizon");

    viewer->spin();
}
#endif

#if edgeFromgood
//test egde
int main()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer!"));
    pcl::PointCloud<PointT>tr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_line(new pcl::PointCloud<PointT>);
    std::string good = "/home/ubuntu/lidar/pp-p1.pcd";
    if (pcl::io::loadPCDFile("/home/ubuntu/lidar/pp-p1.pcd", *cloud) == -1)
    {
        cout << "could not load the ile..." << endl;
    }

    pcl::PointCloud<PointT>::Ptr cloud_part(new pcl::PointCloud<PointT>);
    ProcessPointClouds *ppc = new ProcessPointClouds();
    Eigen::Vector4f minPoint(10,-100,-10, 0);
    Eigen::Vector4f maxPoint(23,90, 0, 0);
    cloud_part = ppc->CropCloud(cloud, minPoint, maxPoint);
    std::cout<<"cloud size: "<<cloud_part->size()<<std::endl;
    cloud_line = ppc->GetEdge(cloud_part);
    std::cout<<"cloud line size: "<<cloud_line->size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud_line, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloud_part, 0, 255, 0);
    viewer->addPointCloud(cloud_part, g, "ori");
    viewer->addPointCloud(cloud_line, r, "line");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "line");

    int n=cloud_line->size();
    for(int i=1;i<n;i++)
    {
        PointT p1 = cloud_line->points[i-1];
        PointT p2 = cloud_line->points[i];
        viewer->addLine(p1, p2, 1,0,0, std::to_string(i), 0);
    }

//    //添加直线
//    pcl::PointXYZ p1, p2;
//    p1.x = 0; p1.y = 0; p1.z = 0;
//    p2.x = 1; p2.y = 1; p2.z = 1;
//    viewer->addLine(p1, p2, 0, 1, 0, "line", 0);
//
//    //添加面normal_x normal_y normal_z d
//    pcl::ModelCoefficients plane;
//    plane.values.push_back(0.0);
//    plane.values.push_back(0.0);
//    plane.values.push_back(1.0);
//    plane.values.push_back(10.0);
//    viewer->addPlane(plane, "plane", 0);
    viewer->spin();
}
#endif

#if edgeFromraw
//test egde from raw
int main()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer!"));
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_line(new pcl::PointCloud<PointT>);
    std::string pcd = "/home/ubuntu/lidar/pp-32.pcd";
    std::string combine = "/home/ubuntu/lidar/poshe1/6pos2052.pcd"; //bad case
    std::string good = "/home/ubuntu/lidar/pp-p1.pcd";
    if (pcl::io::loadPCDFile(pcd, *cloud) == -1)
    {
        cout << "could not load the file..." << endl;
    }


    auto *ppc = new ProcessPointClouds();
    std::cout<<"cloud  size: "<<cloud->size()<<std::endl;
    //viewer->addPointCloud(cloud, "ori1");
    // 1.first get horizon plane return pair(up, down)
    std::pair<PtCdPtr, PtCdPtr> result = ppc->SegmentPlaneHorizon(cloud, 100, 0.3);
    pcl::PointCloud<PointT>::Ptr cloud_part(new pcl::PointCloud<PointT>);
    cloud_part = result.second;
    //cloud_line = result.first;
    std::cout<<"cloud part size: "<<cloud_part->size()<<std::endl;

//    PointT minPt, maxPt;
//    pcl::getMinMax3D(*cloud, minPt, maxPt);
//    std::cout<<"plane min: "<<minPt.x<<" "<<minPt.y<<" "<<minPt.z<<std::endl;
//    std::cout<<"plane max: "<<maxPt.x<<" "<<maxPt.y<<" "<<maxPt.z<<std::endl;
    //2. crop nearby lidar noise points
    Eigen::Vector4f minPoint(10,-100,-10, 0);
    Eigen::Vector4f maxPoint(30,90, 20, 0);
    cloud_part = ppc->CropCloud(cloud_part, minPoint, maxPoint);
    std::cout<<"cloud part crop size: "<<cloud_part->size()<<std::endl;

    //3. get edge
    cloud_line = ppc->GetEdge(cloud_part);
    std::cout<<"cloud line size: "<<cloud_line->size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud_line, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloud_part, 0, 255, 0);
    viewer->addPointCloud(cloud_part, g, "ori");
    viewer->addPointCloud(cloud_line, r, "line");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "line");
//
//    int n=cloud_line->size();
//    for(int i=1;i<n;i++)
//    {
//        PointT p1 = cloud_line->points[i-1];
//        PointT p2 = cloud_line->points[i];
//        viewer->addLine(p1, p2, 1,0,0, std::to_string(i), 0);
//    }

    viewer->spin();
}
#endif