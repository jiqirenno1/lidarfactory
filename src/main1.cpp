//
// Created by ubuntu on 2020/11/3.
//

#include <iostream>
#include "rs_driver/api/lidar_driver.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
using namespace robosense::lidar;

robosense::lidar::Queue<robosense::lidar::PointCloudMsg<pcl::PointXYZ>> msg_queue;
//pcl::visualization::PCLVisualizer::Ptr viewer =pcl::shared_ptr<pcl::visualization::PCLVisualizer>(
//    new pcl::visualization::PCLVisualizer("RS LIDAR 16 Viewer"));



void show(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    if(msg_queue.size()==0)
    {
        return;
    }
    while(msg_queue.size()>0)
    {
        auto msg = msg_queue.front();
        msg_queue.popFront();

        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        point_cloud->header.stamp = msg.timestamp;
        point_cloud->header.seq = msg.seq;
        point_cloud->header.frame_id = msg.frame_id;
        point_cloud->width = msg.width;
        point_cloud->height = msg.height;
        point_cloud->is_dense = msg.is_dense;

        size_t n_points = msg.point_cloud_ptr->size();
        point_cloud->points.resize(n_points);
        std::cout<<"** size: "<<n_points<<"  **seq: "<<point_cloud->header.seq<<std::endl;
        for(size_t i=0;i<n_points;i++)
        {
            point_cloud->points.at(i).x = msg.point_cloud_ptr->at(i).x;
            point_cloud->points.at(i).y = msg.point_cloud_ptr->at(i).y;
            point_cloud->points.at(i).z = msg.point_cloud_ptr->at(i).z;
        }

//        if(n_points==28800)
//        {
//            std::cout<<"save PCD!"<<std::endl;
//            pcl::io::savePCDFile ("/home/ubuntu/lidar/configuration_data/test_pcd_pcl.pcd", *point_cloud);
//        }
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
//        renderPointCloud(point_cloud, msg.frame_id+std::to_string(msg.seq));
        viewer->addPointCloud<pcl::PointXYZI> (point_cloud, msg.frame_id+std::to_string(msg.seq));
        viewer->spinOnce();



    }
}
struct PointXYZI  ///< user defined point type
{
    float x;
    float y;
    float z;
    float intensity;
};

/**
 * @brief The point cloud callback function. This function will be registered to lidar driver.
 *              When the point cloud message is ready, driver can send out messages through this function.
 * @param msg  The lidar point cloud message.
 */
void pointCloudCallback(const PointCloudMsg<pcl::PointXYZ>& msg)
{
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the message and process it in another thread is recommended*/
    RS_MSG << "msg: " << msg.seq <<" msg.width: "<<msg.width <<" point cloud size: " << msg.point_cloud_ptr->size() << RS_REND;
    msg_queue.push(std::move(msg));
}

/**
 * @brief The exception callback function. This function will be registered to lidar driver.
 * @param code The error code struct.
 */
void exceptionCallback(const Error& code)
{
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the error message and process it in another thread is recommended*/
    RS_WARNING << "Error code : " << code.toString() << RS_REND;
}

int main(int argc, char* argv[])
{
    RS_TITLE << "------------------------------------------------------" << RS_REND;
    RS_TITLE << "            RS_Driver Core Version: V " << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
             << RSLIDAR_VERSION_PATCH << RS_REND;
    RS_TITLE << "------------------------------------------------------" << RS_REND;

    LidarDriver<pcl::PointXYZ> driver;  ///< Declare the driver object

    RSDriverParam param;                  ///< Create a parameter object
    param.input_param.msop_port = 6699;   ///< Set the lidar msop port number, the default is 6699
    param.input_param.difop_port = 7788;  ///< Set the lidar difop port number, the default is 7788
    param.lidar_type = LidarType::RS16;   ///< Set the lidar type. Make sure this type is correct
    param.print();

    driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function into the driver
    driver.regRecvCallback(pointCloudCallback);      ///< Register the point cloud callback function into the driver
    if (!driver.init(param))                         ///< Call the init function and pass the parameter
    {
        RS_ERROR << "Driver Initialize Error..." << RS_REND;
        return -1;
    }
    driver.start();  ///< The driver thread will start
    RS_DEBUG << "RoboSense Lidar-Driver Linux online demo start......" << RS_REND;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("RS LIDAR 16 Viewer"));
    viewer->addCoordinateSystem();

    while (true)
    {
//        sleep(1);
        show(viewer);
    }
    return 0;

}