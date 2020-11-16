//
// Created by ubuntu on 2020/11/4.
//
#include <pcl/common/common.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "ProcessPointClouds.h"
#include <cmath>
#include <pcl/common/transforms.h>
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("pcd viewer"));
    ProcessPointClouds *ProcessorI = new ProcessPointClouds();

    viewer->addCoordinateSystem();

    std::string file0 = "/home/ubuntu/CLionProjects/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000001.pcd";
    std::string file ="/home/ubuntu/lidar/data/test_400.pcd";
    std::string cropFile ="/home/ubuntu/lidar/data/myCrop2.pcd";
    pcl::io::loadPCDFile(cropFile, *cloud);
//  transform
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    double theta = M_PI/180;
    transform.translation()<<0.0,0.0,0.0;
    transform.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));
    std::cout<<transform.matrix()<<std::endl;
    std::cout<<"cloud size: "<<cloud->size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr combine(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *combine, transform);
    std::cout<<"combine size: "<<combine->size()<<std::endl;

    *combine = *combine + *cloud;
    std::cout<<"then combine size: "<<combine->size()<<std::endl;
    //transform
/*
   size_t N = cloud->size();
   pcl::PointXYZ point;
   std::vector<pcl::PointCloud<pcl::PointXYZ>> CloudScans(16);
   for(size_t i=0;i<N;i++)
   {
       point.x = cloud->points[i].x;
       point.y = cloud->points[i].y;
       point.z = cloud->points[i].z;

       float angle=atan(point.z/sqrt(point.x*point.x+point.y*point.y))*180/M_PI;
       int scanID=0;
       scanID = int((angle+15)/2+0.5);
       std::cout<<"scanID: "<<scanID<<std::endl;
       CloudScans[scanID].push_back(point);

   }
    pcl::PointCloud<pcl::PointXYZ>::Ptr show(new pcl::PointCloud<pcl::PointXYZ>);
   for(int i=0;i<CloudScans.size();i++)
   {
       std::cout<<"scans:"<<i<<" "<<CloudScans[i].size()<<std::endl;
       *show += CloudScans[i];
   }
   */





//    PtCdPtr crop = ProcessorI->CropCloud(cloud, Eigen::Vector4f(10, -3, -8, 1), Eigen::Vector4f(30, 6, 8,1));
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> colorop, 0, 255, 0);
//    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb);
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> rgb(combine, "z");
   viewer->addPointCloud(combine, rgb, "cloud0");
//     viewer->addPointCloud(CloudScans[0], "cloud1");
//    std::pair<PtCdPtr, PtCdPtr> result = ProcessorI->SegmentPlaneWithNormal(cloud, 50, 0.2);
//    std::pair<PtCdPtr, PtCdPtr> result = ProcessorI->SegmentCylinder(cloud);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(result.first, 0, 255, 0);
//    viewer->addPointCloud(result.first, color, "crop");
//    pcl::PointCloud<pcl::Normal>::Ptr nor = ProcessorI->GetNormals(crop);
//    std::cout<<"nor siz"<<crop->points[0]<<" "<<nor->points[0].normal<<std::endl;
//    std::cout<<"nor siz"<<nor->points[0].curvature<<" "<<nor->points[0]<<std::endl;
//    viewer->addPointCloudNormals<PointT, pcl::Normal>(crop, nor,50, 0.01, "normals");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "normals");
//    pcl::io::savePCDFile ("/home/ubuntu/lidar/data/myCrop2.pcd", *crop);
//      pcl::PolygonMesh tri = ProcessorI->CalConvexHull(cloud);
//      viewer->addPolygonMesh(tri, "tri");
//     PtCdPtr filter = ProcessorI->BilateralFilter(cloud);
//        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(filter, 0, 255, 0);
//    viewer->addPointCloud(filter, color, "crop");
//   pcl::PolygonMesh mesh = ProcessorI->GreedyTriangle(cloud);
//   viewer->addPolygonMesh(mesh, "mesh");


    viewer->spin();
}