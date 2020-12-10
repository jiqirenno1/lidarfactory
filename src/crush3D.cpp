//
// Created by ubuntu on 2020/11/30.
//

#include <pcl/common/common.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include "ProcessPointClouds.h"

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));

void viewerplus(const pcl::visualization::PCLVisualizer::Ptr& viewer1, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
                const std::string& txt, double size, double r, double g, double b)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointT> c(cloud1, r, g, b);
    viewer1->addPointCloud(cloud1, c, txt);
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, txt);

}
float getZ(float x)
{
    if(-50<=x&&x<=50)
    {
        return 30;
    }
    else if(x<-50)
    {
        return (80+x);
    }

    else if(x>50)
    {
        return (80-x);
    }

}

bool IsBoundary(pcl::PointXYZ &p)
{
    if(p.y>50||p.y<-50)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void IsCrush(pcl::PointXYZ &p1, pcl::PointXYZ &p2, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, std::string &msg)
{
    cloud1->clear();
    if(sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z))<4)
    {
        msg =  "car will crush person!";
        cloud1->points.push_back(p1);
    }
    if(p1.y<-47||p1.y>47)
    {
        cloud1->points.push_back(p1);
        msg =  "danger!";

    }
    if(p2.y<-47||p2.y>47)
    {
        cloud1->points.push_back(p2);
        msg =  "danger!";

    }

}
pcl::PointCloud<pcl::PointXYZ>::Ptr GetLine(pcl::PointXYZ &p)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    float x = p.x;

    float t = x-6;
    while(t<x+5)
    {
        pcl::PointXYZ p1;
        pcl::PointXYZ p2;
        p1.x = t;
        p1.y = -t*t/10+40;
        p1.z = 35;
        if(p1.y<=50&&p1.y>=-50)
        {
            cloud->points.push_back(p1);
        }

        p2.x = t;
        p2.y = t*t/10-40;;
        p2.z = 35;
        if(p2.y<=50&&p2.y>=-50)
        {
            cloud->points.push_back(p2);
        }

        if(t<=x)
        {
            t+=0.1;
        }
        else
        {
            t+=0.5;
        }

    }
    return cloud;
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
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lineP(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addCoordinateSystem();
    viewer->setCameraPosition(-208.036117,-262.116145,387.731762,0.208293,0.739504,0.640115);





    for(float i=-100;i<=100;i+=1)
    {
        for(float j=-80;j<=80;j+=1)
        {
            pcl::PointXYZ p;
            p.x = i;
            p.y = j;
            p.z = getZ(j);
            cloud0->points.push_back(p);
            if(p.z==30&&(p.y==-50||p.y==50))
            {
                line->points.push_back(p);
            }
        }
    }
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(cloud0, "z");
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(cloud0, 0, 255, 255);
//    viewer->addPointCloud(cloud0, g);
    viewerplus(viewer, cloud0, "", 1, 0, 255, 255);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(line, 255, 0, 0);
//    viewer->addPointCloud(line, r, "line");
    viewerplus(viewer, line, "line", 1, 255,0,0);
    pcl::PointXYZ p1{-29,0,35};
    pcl::PointXYZ p2{0,10,35};
    pcl::PointXYZ p3{-100,80,35};

    lineP = GetLine(p1);
    viewerplus(viewer, lineP, "lineP", 3, 0,0,100);
//    viewer->addPointCloud(lineP, "lineP");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "lineP");

    viewer->addSphere(p1, 3, "person");
    viewer->addSphere(p2, 3, "car");

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

    for(float i=-30;i<=80;i+=1)
    {
        p1.x = i;
        p1.y = -i*i/10+40;

        if(!IsBoundary(p1))
        {
            viewer->updateSphere(p1, 5, 1,0,0, "person");
            viewer->removeText3D("1");
            viewer->addText3D("person", p1, 8, 1,0, 0, "1");
            lineP = GetLine(p1);
            viewer->updatePointCloud(lineP, "lineP");
        }
        else
        {
//            viewer->removeText3D("warn1");
//            viewer->addText3D("person danger!", p1, 8, 0, 0, 1, "warn1");

        }

        p2.x = i;
        p2.y = i*i/10-40;

        if(!IsBoundary(p2))
        {
            viewer->updateSphere(p2, 5, 0,1,0,"car");
            viewer->removeText3D("2");
            viewer->addText3D("car", p2, 8, 0, 1, 0, "2");
        }
        else
        {
//            viewer->removeText3D("warn2");
//            viewer->addText3D("car danger!", p2, 8, 0, 0, 1, "warn2");
            break;
        }

        std::string msg;
        IsCrush(lineP->points[lineP->size()-2], lineP->points[lineP->size()-1], pts, msg);
        viewer->removePointCloud("pts");
        viewerplus(viewer, pts, "pts", 20, 0, 0, 255);
        viewer->removeText3D("msg");
        viewer->addText3D(msg, lineP->points[lineP->size()-1], 5, 0, 0, 1, "msg");

        viewer->spinOnce();
        sleep(1);
    }

    auto pro = new ProcessPointClouds();
//    auto mesh = pro->GreedyTriangle(cloud0);
//    viewer->addPolygonMesh(mesh);
    viewer->spin();

}
