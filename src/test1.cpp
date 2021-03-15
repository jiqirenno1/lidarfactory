//
// Created by ubuntu on 2021/3/10.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include "ProcessPointClouds.h"
#include "utils/Httplib.h"

using namespace std;

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
    client->Send(im, "82.156.16.242", 9081, "/my", detaX, detaY, minX, minY);
}
//test cloud2mat
int main1()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("look"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd = "/home/ubuntu/lidar/combine.pcd";
    pcl::io::loadPCDFile(pcd, *cloud);
    std::shared_ptr<ProcessPointClouds> ppc = std::make_shared<ProcessPointClouds>();

    cloud = ppc->DownSampleCloud(cloud, 0.6);
    std::vector<float> params = ppc->GetFov(cloud);
    float xmin = params[0];
    float ymin = params[1];
    float xlen = params[2];
    float ylen = params[3];

    float detaX = 0.3, detaY = 0.3;
    cv::Mat img;
    ppc->Cloud2Mat(cloud, img, detaX, detaY, xmin, ymin, xlen, ylen);
    cv::imwrite("/home/ubuntu/lidar/out1.png", img);
    ppc->Mat2Cloud(img, detaX, detaY,xmin, ymin, cloudout);

    viewer->addPointCloud(cloud, "origin");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloudout, 0,255,0);
    viewer->addPointCloud(cloudout, g, "out");
    viewer->spin();
}
