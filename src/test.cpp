//
// Created by ubuntu on 2020/11/4.
//
#include <pcl/common/common.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "ProcessPointClouds.h"
#include <cmath>
#include <pcl/common/transforms.h>

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

int main()
{
    std::string file = "/home/ubuntu/lidar/combine.pcd";
    std::string file1 = "/home/ubuntu/lidar/sml.pcd";
    PtCdPtr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(file, *cloud);
//    std::cout<<"mesh: "<<cloud->height<<"\n";

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addCoordinateSystem();
    auto proI = new ProcessPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> rgb(cloud, "z");

    //PtCdPtr cloudP = proI->PassThrough(cloud, "z",-1.3, -0.3);
    PtCdPtr cloudP = cloud;
    std::cout<<"before : "<<cloud->size()<<"\n";
    PtCdPtr res = proI->RemovalOutlier(cloudP);
    std::cout<<"after : "<<res->size()<<"\n";

    std::pair<PtCdPtr, PtCdPtr> result = proI->SegmentPlane(cloudP, 50, 0.1);
//    viewer->addPointCloud(result.first, "plane");
    std::cout<<"size 1 : "<<result.first->size()<<"\n";
    pcl::PointCloud<pcl::PointNormal> outCloud = proI->Smoothing(result.first);
    std::cout<<"size 2 : "<<outCloud.size()<<"\n";

    PtCdPtr sm(new pcl::PointCloud<PointT>);
    size_t ss = outCloud.size();
    sm->resize(ss);
    for(size_t i=0;i<ss;i++)
    {
        sm->points[i].x = outCloud.points[i].x;
        sm->points[i].y = outCloud.points[i].y;
        sm->points[i].z = outCloud.points[i].z;
    }
    std::cout<<"size 3 : "<<sm->size()<<"\n";

//    viewer->addPointCloud(cloudP, r, "cloudall");
     viewer->addPointCloud(cloudP, "cloud");
//    PtCdPtr smdown = proI->DownSampleCloud(sm, 0.3);
    pcl::PolygonMesh mesh=proI->GreedyTriangle(res);
    viewer->addPolygonMesh(mesh, "mesh");
//    std::cout<<"mesh: "<<mesh.polygons.size()<<"\n";
      viewer->spin();

}
int mainlp()
{
    std::string file0 = "/home/ubuntu/lidar/poshe1/6pos2048.pcd";
    PtCdPtr cloud1(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(file0, *cloud1);

    std::string datapath = "/home/ubuntu/lidar/poshe1";
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{datapath},
                                               boost::filesystem::directory_iterator{});

    std::sort(paths.begin(), paths.end());
    PtCdPtr combine(new pcl::PointCloud<PointT>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("pcd viewer"));
    viewer->addCoordinateSystem();
    int lastP=0;
    for(auto &e:paths)
    {
        PtCdPtr cloud(new pcl::PointCloud<PointT>);

        std::cout<<e<<std::endl;
        const std::string& name = e.string();
        pcl::io::loadPCDFile(name, *cloud);
        auto pos = name.find_last_of('/');
        auto leaf = name.substr(pos+5, 4);
        int p = atoi(leaf.c_str());
        if((p-lastP)<2)
        {
            lastP = p;
            continue;
        }
        lastP = p;
        std::cout<<p<<"\n";
        double theta1 = M_PI/180*(p-2048)/4096*360;
        *combine+=*lidar2base(cloud, theta1);
        std::cout<<"then combine size: "<<combine->size()<<std::endl;

        viewer->removePointCloud("cloud");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> r(combine,0, 255, 0);
        viewer->addPointCloud(combine, r, "cloud");
        viewer->spinOnce();
        //sleep(1);

    }
    ProcessPointClouds *proI =new ProcessPointClouds();
    combine = proI->DownSampleCloud(combine, 0.3);
    std::cout<<"then combine size: "<<combine->size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud1,0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> b(combine,0, 0,255);
//    viewer->addPointCloud(cloud1, r, "cloud1");
//    viewer->addPointCloud(combine, b, "cloud");

//    pcl::io::savePCDFile ("/home/ubuntu/lidar/combine.pcd", *combine);
    pcl::PolygonMesh mesh= proI->GreedyTriangle(combine);
    std::cout<<"mesh: "<<mesh.polygons.size()<<"\n";
    viewer->removePointCloud("cloud");
    viewer->addPolygonMesh(mesh, "mesh");


    viewer->spin();


}
int main1()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("pcd viewer"));
    ProcessPointClouds *ProcessorI = new ProcessPointClouds();

    viewer->addCoordinateSystem();

    std::string file0 = "/home/ubuntu/lidar/pos/6pos2048.pcd";
    std::string file ="/home/ubuntu/lidar/pos/6pos2150.pcd";
    std::string cropFile ="/home/ubuntu/lidar/data/myCrop2.pcd";
    pcl::io::loadPCDFile(file0, *cloud);
    pcl::io::loadPCDFile(file, *cloud1);

    PtCdPtr combine0(new pcl::PointCloud<PointT>);
    PtCdPtr combine1(new pcl::PointCloud<PointT>);
    PtCdPtr combine(new pcl::PointCloud<PointT>);

    double theta0 = 0;
    double theta1 = M_PI/180*(2150-2048)/4096*360;
    std::cout<<"Theta1: "<<theta1<<std::endl;
    combine0 = lidar2base(cloud, theta0);
    combine1 = lidar2base(cloud1, theta1);
//    *combine = *combine0+*combine1;
   *combine = *cloud+*cloud1;
    std::cout<<"then combine size: "<<combine->size()<<std::endl;




////  transform
//    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
//    std::cout<<transform.matrix()<<std::endl;
//    double theta = M_PI/2;
//    transform.translation()<<0.0,0.0,0.0;
//    transform.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
//    std::cout<<transform.matrix()<<std::endl;
//    std::cout<<"cloud size: "<<cloud->size()<<std::endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr combine(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::transformPointCloud(*cloud, *combine, transform);
//    std::cout<<"combine size: "<<combine->size()<<std::endl;
//
//    *combine = *combine + *cloud;
//    std::cout<<"then combine size: "<<combine->size()<<std::endl;
//    //transform
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
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud1,255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloud,0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> b(combine1,0, 0, 255);
   viewer->addPointCloud(combine1, b, "cloud");
    viewer->addPointCloud(combine,"cloud0");
//    viewer->addPointCloud(combine1, b,"cloud1");
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