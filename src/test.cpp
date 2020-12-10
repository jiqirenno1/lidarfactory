//
// Created by ubuntu on 2020/11/4.
//
#include <pcl/common/common.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "ProcessPointClouds.h"
#include <cmath>
#include <pcl/common/transforms.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));

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

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* args) {

    if (event.getKeySym() == "r" && event.keyDown()) {
        std::cout <<"t!"<<std::endl;
        pcl::visualization::Camera camera;
        viewer->getCameraParameters(camera);
        printf("%lf,%lf,%lf,", camera.pos[0], camera.pos[1], camera.pos[2]);
        printf("%lf,%lf,%lf\n", camera.view[0], camera.view[1], camera.view[2]);

    }
}

int mainmy()
{
    std::string file = "/home/ubuntu/lidar/combine.pcd";
    std::string file1 = "/home/ubuntu/lidar/sml.pcd";
    PtCdPtr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(file, *cloud);
//    std::cout<<"mesh: "<<cloud->height<<"\n";

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addCoordinateSystem();
    auto proI = new ProcessPointClouds();

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> rgb(cloud, "z");

    //PtCdPtr cloudP = proI->PassThrough(cloud, "x",8.0, 20.0);
    //PtCdPtr cloudP = cloud;
    PtCdPtr cloudP = proI->DownSampleCloud(cloud, 0.6);
    std::cout<<"before : "<<cloud->size()<<"\n";
    PtCdPtr res = proI->RemovalOutlier(cloudP);
    std::cout<<"after : "<<res->size()<<"\n";

    std::pair<PtCdPtr, PtCdPtr> result = proI->SegmentPlane(res, 50, 0.1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloudP, 0, 255, 0);
    viewer->addPointCloud(cloudP, r, "cloudall");

//    PtCdPtr net = proI->EstimateUpNet(cloudP);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(net, 255, 0, 0);
//    viewer->addPointCloud(net,  g, "net");
//
    pcl::PointCloud<pcl::Normal>::Ptr nn = proI->GetNormals(cloudP);
//    PtCdPtr plane(new pcl::PointCloud<PointT>);
//    for(size_t i=0;i<nn->size();i++)
//    {
//        if(nn->points[i].curvature<=0.1)
//        {
//            plane->points.push_back(cloudP->points[i]);//        }
//    }
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> p(plane, 0, 255, 255);
//    viewer->addPointCloud(plane, p, "pplane");

    std::vector<PtCdPtr> cls = proI->RegionGrowing(cloudP);
    //viewer->addPointCloudNormals<PointT, pcl::Normal>(cloudP, nn, 2, 0.3); //每2个点显示一个及每个法线的长度0.3
    for(auto & cl:cls)
    {
        std::cout<<"cls: "<<cl->size()<<"\n";
    }

    viewer->addPointCloud(cls[0],  "plane");
    //pcl::io::savePCDFile ("/home/ubuntu/lidar/plane.pcd", *res);

    PtCdPtr bound = proI->EstimateBoundary(cls[0]);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> b(bound, 255, 0, 0);
    viewer->addPointCloud(bound, b, "boundry");

//    pcl::PointCloud<pcl::PointNormal> outCloud = proI->Smoothing(result.first);
//    std::cout<<"size 2 : "<<outCloud.size()<<"\n";
//
//    PtCdPtr sm(new pcl::PointCloud<PointT>);
//    size_t ss = outCloud.size();
//    sm->resize(ss);
//    for(size_t i=0;i<ss;i++)
//    {
//        sm->points[i].x = outCloud.points[i].x;
//        sm->points[i].y = outCloud.points[i].y;
//        sm->points[i].z = outCloud.points[i].z;
//    }
//    std::cout<<"size 3 : "<<sm->size()<<"\n";

//    viewer->addPointCloud(cloudP, r, "cloudall");

//    PtCdPtr smdown = proI->DownSampleCloud(sm, 0.3);
//    pcl::PolygonMesh mesh=proI->GreedyTriangle(result.second);
//    viewer->addPolygonMesh(mesh, "mesh");
//    std::cout<<"mesh: "<<mesh.polygons.size()<<"\n";
      viewer->spin();

}
int main()
{
    viewer->setFullScreen(false);
    std::string file0 = "/home/ubuntu/lidar/poshe1/6pos2048.pcd";
    PtCdPtr cloud1(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(file0, *cloud1);

    std::string datapath = "/home/ubuntu/lidar/poshe1";
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{datapath},
                                               boost::filesystem::directory_iterator{});

    std::sort(paths.begin(), paths.end());
    PtCdPtr combine(new pcl::PointCloud<PointT>);
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("pcd viewer"));

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    viewer->setCameraPosition(-484.391530,91.717888,392.964559,0.608781,0.033757,0.792620);
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
        if((p-lastP)<1)
        {
            lastP = p;
            continue;
        }
        lastP = p;
        std::cout<<p<<"\n";
        double theta1 = M_PI/180*(p-2048)/4096*360;
        *combine+=*lidar2base(cloud, theta1);
        //std::cout<<"then combine size: "<<combine->size()<<std::endl;

        viewer->removePointCloud("cloud");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> r(combine,0, 255, 0);
        viewer->addPointCloud(combine, r, "cloud");
        viewer->spinOnce();
        if(p<2080)
        {
            sleep(1);

        }


    }
    ProcessPointClouds *proI =new ProcessPointClouds();
    PtCdPtr combine1 = proI->DownSampleCloud(combine, 0.3);
    combine = proI->DownSampleCloud(combine, 0.6);
    viewer->removePointCloud("cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(combine,0, 255, 0);
    viewer->addPointCloud(combine, green, "cloud");

    std::vector<PtCdPtr> cls = proI->RegionGrowing(combine);
    viewer->addPointCloud(cls[0],  "plane");
    PtCdPtr bound = proI->EstimateBoundary(cls[0]);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> b(bound, 255, 0, 0);
    viewer->addPointCloud(bound, b, "boundry");
    viewer->spinOnce();
    viewer->saveScreenshot("shot.jpg");
    sleep(5);

    //std::cout<<"then combine size: "<<combine->size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud1,0, 255, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> b(combine,0, 0,255);
//    viewer->addPointCloud(cloud1, r, "cloud1");
//    viewer->addPointCloud(combine, b, "cloud");

    //pcl::io::savePCDFile ("/home/ubuntu/lidar/combine0.pcd", *combine);
    pcl::PolygonMesh mesh= proI->GreedyTriangle(combine1);
    std::cout<<"mesh: "<<mesh.polygons.size()<<"\n";
    viewer->removeAllPointClouds();
    viewer->addPolygonMesh(mesh, "mesh");


    viewer->spin();


}