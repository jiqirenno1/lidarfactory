//
// Created by ubuntu on 2020/11/3.
//

#include <iostream>
#include "rs_driver/api/lidar_driver.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "SMSCL.h"
using namespace robosense::lidar;

SMSCL sm;

robosense::lidar::Queue<std::pair<robosense::lidar::PointCloudMsg<pcl::PointXYZ>, int>> msg_queue;

//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* cloud_void)
//{
//
//    pcl::PointCloud<pcl::PointXYZ> *cloud = static_cast<pcl::PointCloud<pcl::PointXYZ>*>(cloud_void);
//    if(event.getKeySym()=="s"&& event.keyDown())
//    {
//        if(1)
//        {
//            std::cout<<"** size: "<<cloud->size()<<"  **width*height: "<<cloud->width*cloud->height<<std::endl;
//            std::cout<<"save PCD!"<<std::endl;
//            pcl::io::savePCDFile ("/home/ubuntu/lidar/configuration_data/s_pcd_pcl.pcd", *cloud);
//        }
//
//    }
//
//
//}

int getPos()
{
    int pos;
    if(sm.FeedBack(1)!=-1){
        pos = sm.ReadPos(-1);
        std::cout<< "pos ="<<pos<<std::endl;
    }
    return pos;
}

void run()
{
    s16 pos0 = 2048;
    s16 step = 20;
    while(1) {

        s16 pos1 = pos0 + step;
        if (pos1 >= 2248 || pos1 <= 2048)
        {
            step = -step;
        }
        sm.WritePosEx(1, pos1, 200, 3);
        usleep(700*1000);
        if(sm.FeedBack(1)!=-1){
            std::cout<< "pos"<<pos1<<" ="<<sm.ReadPos(-1)<<std::endl;
        }
        pos0=pos1;
    }
}

void show(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    int pos = getPos();
    if(msg_queue.size()==0)
    {
        return;
    }
    while(msg_queue.size()>0)
    {
        auto res = msg_queue.front();
        msg_queue.popFront();
        auto msg = res.first;
        int pos = res.second;

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        point_cloud->header.stamp = msg.timestamp;
        point_cloud->header.seq = msg.seq;
        point_cloud->header.frame_id = msg.frame_id;
        point_cloud->width = msg.width;
        point_cloud->height = msg.height;
        point_cloud->is_dense = msg.is_dense;

        size_t n_points = msg.point_cloud_ptr->size();
        point_cloud->points.resize(n_points);
        std::cout<<"** size: "<<n_points<<"  **width*height: "<<point_cloud->width*point_cloud->height<<std::endl;
        std::cout<<"** size: "<<n_points<<"  **seq: "<<point_cloud->header.seq<<std::endl;
        for(size_t i=0;i<n_points;i++)
        {
            point_cloud->points.at(i).x = msg.point_cloud_ptr->at(i).x;
            point_cloud->points.at(i).y = msg.point_cloud_ptr->at(i).y;
            point_cloud->points.at(i).z = msg.point_cloud_ptr->at(i).z;
        }

        if(point_cloud->header.seq%1==0)
        {
            std::cout<<"save PCD!"<<std::endl;
            pcl::io::savePCDFile ("/home/ubuntu/lidar/poshe1/6pos"+ std::to_string(pos)+".pcd", *point_cloud);
        }
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
//        renderPointCloud(point_cloud, msg.frame_id+std::to_string(msg.seq));
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(point_cloud, "z");
        viewer->addPointCloud<pcl::PointXYZ> (point_cloud, rgb, msg.frame_id+std::to_string(msg.seq));
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
    int pos = getPos();
    std::pair<robosense::lidar::PointCloudMsg<pcl::PointXYZ>, int> res(std::move(msg), pos);
    msg_queue.push(res);

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

    std::string path = "/dev/ttyUSB0";
    std::cout<< "serial:"<<path<<std::endl;
    if(!sm.begin(115200, path.c_str())){
        std::cout<< "Failed to init smscl motor!"<<std::endl;
        return 0;
    }
    sm.WritePosEx(1, 2048, 1000, 10);
    usleep(2270*1000);
    sm.FeedBack(1);
    std::cout<< " init pos ="<<sm.ReadPos(-1)<<std::endl;
    std::thread t(run);

    RS_TITLE << "------------------------------------------------------" << RS_REND;
    RS_TITLE << "            RS_Driver Core Version: V " << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
             << RSLIDAR_VERSION_PATCH << RS_REND;
    RS_TITLE << "------------------------------------------------------" << RS_REND;

    LidarDriver<pcl::PointXYZ> driver;  ///< Declare the driver object

    RSDriverParam param;                  ///< Create a parameter object
    param.input_param.device_ip = "192.168.1.200";
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


//
//    while (true)
//    {
////        sleep(1);
//        show(viewer);
//    }
    while(!viewer->wasStopped())
    {
        show(viewer);
        viewer->spinOnce();

    }
    t.join();
    return 0;


}