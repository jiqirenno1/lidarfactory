//
// Created by ubuntu on 2021/3/10.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include "ProcessPointClouds.h"
#include "utils/pistache.h"

using namespace std;

int main()
{
    pistache *client = new pistache();
    string im1 = "/home/ubuntu/CLionProjects/LiDAR-Point-Cloud-Compression/build/out.png";
   string im = "/home/ubuntu/lidar/out0.001.png";
    string page = "http://localhost:9080/my";
    float detaX = 0.2;
    float detaY = 0.3;
    float minX = 8;
    float minY = 10;
    client->pistacheSend(im, page, detaX, detaY, minX, minY);
}
//test cloud2mat
int main1()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("look"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd = "/home/ubuntu/lidar/poshe1/6pos2048.pcd";
    pcl::io::loadPCDFile(pcd, *cloud);
    ProcessPointClouds *ppc = new ProcessPointClouds();

    cloud = ppc->DownSampleCloud(cloud, 0.6);
    std::vector<float> params = ppc->GetFov(cloud);
    float xmin = params[0];
    float ymin = params[1];
    float xlen = params[2];
    float ylen = params[3];

    float detaX = 0.3, detaY = 0.6;
    cv::Mat img;
    ppc->Cloud2Mat(cloud, img, detaX, detaY, xmin, ymin, xlen, ylen);
    cv::imwrite("/home/ubuntu/lidar/out.png", img);
    ppc->Mat2Cloud(img, detaX, detaY,xmin, ymin, cloudout);

    viewer->addPointCloud(cloud, "origin");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloudout, 0,255,0);
    viewer->addPointCloud(cloudout, g, "out");
    viewer->spin();
}
