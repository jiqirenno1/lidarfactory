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
//test sm
int mainm()
{
    shared_ptr<SM45> sm_ptr = make_shared<SM45>();
    std::string port  = "/dev/ttyUSB0";
    sm_ptr->init(port, 115200);
    sm_ptr->start();

}
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
//test cloud2mat
int main1()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("look"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd = "/home/ubuntu/lidar/2/combine.pcd";
    std::string pcd1 = "/home/ubuntu/lidar/6pos2048.pcd";
    pcl::io::loadPCDFile(pcd1, *cloud);
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
    cv::Mat img;
    ppc->Cloud2Mat(cloud, img, detaX, detaY, xmin, ymin, xlen, ylen);
    cv::imwrite("/home/ubuntu/lidar/out2.png", img);
    ppc->Mat2Cloud(img, detaX, detaY,xmin, ymin, cloudout);

    viewer->addPointCloud(cloud, "origin");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloudout, 0,255,0);
    viewer->addPointCloud(cloudout, g, "out");
    viewer->spin();
}

//test horizonplanesegment
int mainh()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer!"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<PointT>);
    std::string pcd = "/home/ubuntu/lidar/2/combine.pcd";
    pcl::io::loadPCDFile(pcd, *cloud);


    //ProcessPointClouds *ppc = new ProcessPointClouds;
    std::shared_ptr<ProcessPointClouds> ppc_ptr = std::make_shared<ProcessPointClouds>();

    std::cout<<"cloud size: "<<cloud->size()<<std::endl;
    cloud = ppc_ptr->DownSampleCloud(cloud, 0.1);
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
    plane = result1.first;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(plane, 0, 255, 0);
    viewer->addPointCloud(plane, g, "horizon");

    viewer->spin();
}
