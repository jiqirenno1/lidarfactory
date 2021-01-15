//
// Created by ubuntu on 2021/1/14.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>
#include "sdf/sdf.h"
#include "ProcessPointClouds.h"
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdPtr;

PtCdPtr lidar2base(PtCdPtr cloud, double theta)
{
    PtCdPtr resCloud (new pcl::PointCloud<PointT>);
    Eigen::Affine3d Tsl = Eigen::Affine3d::Identity();
    Tsl.translation()<<0.0,0.0,0.033;
//    std::cout<<"Tsl: "<<Tsl.matrix()<<std::endl;
    Eigen::Affine3d Tbs = Eigen::Affine3d::Identity();
    Tbs.translation()<<0.0,0.0,0.04;
    Tbs.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));
//    std::cout<<"Tbs: "<<Tbs.matrix()<<std::endl;

    Tbs = Tbs*Tsl;

    pcl::transformPointCloud(*cloud, *resCloud, Tbs);

    return resCloud;

}

int main1()
{
    float width = 50;
    float height = 50;
    float depth = 50;
    int m= 500;

    Eigen::Vector3f origin(0.0, -25.0, -15.0);
    sdf graph;
    graph.init(m, width, height, depth, origin);

    std::string file = "/home/ubuntu/lidar/combine.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file, *cloud);

    graph.update(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
    cloudout = graph.get_result();
    std::cout<<cloudout->size()<<std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("sdf viewer!"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> b(cloudout, 0, 0, 255);
    viewer->addPointCloud(cloudout, b, "tsdf");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> r(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, r, "init");

    viewer->spin();

}

int main()
{
    float width = 50;
    float height = 50;
    float depth = 50;
    int m= 500;

    Eigen::Vector3f origin(0.0, -25.0, -15.0);
    sdf graph;
    graph.init(m, width, height, depth, origin);

    std::string file = "/home/ubuntu/lidar/combine.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file, *cloud);

    std::string datapath = "/home/ubuntu/lidar/poshe1";
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{datapath},
                                               boost::filesystem::directory_iterator{});

    std::sort(paths.begin(), paths.end());
    PtCdPtr combine(new pcl::PointCloud<PointT>);

    int lastP=0;
    for(auto &e:paths) {
        PtCdPtr cloud(new pcl::PointCloud<PointT>);

        std::cout << e << std::endl;
        const std::string &name = e.string();
        pcl::io::loadPCDFile(name, *cloud);
        auto pos = name.find_last_of('/');
        auto leaf = name.substr(pos + 5, 4);
        int p = atoi(leaf.c_str());
        if ((p - lastP) < 1) {
            lastP = p;
            continue;
        }
        lastP = p;
        std::cout << p << "\n";
        double theta1 = M_PI / 180 * (p - 2048) / 4096 * 360;

        graph.update(lidar2base(cloud, theta1));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
    cloudout = graph.get_result();
    std::cout<<cloudout->size()<<std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("sdf viewer!"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> b(cloudout, 0, 0, 255);
    viewer->addPointCloud(cloudout, b, "tsdf");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> r(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, r, "init");

    viewer->spin();

}