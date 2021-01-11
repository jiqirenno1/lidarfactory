//
// Created by ubuntu on 2021/1/4.
//

#include <iostream>
#include "rs_driver/api/lidar_driver.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "SMSCL.h"
#include <pcl/common/transforms.h>
using namespace robosense::lidar;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdPtr;

pcl::visualization::PCLVisualizer::Ptr pcl_viewer;
PtCdPtr combine;
//PtCdPtr combine1;
std::mutex mex_viewer;
std::shared_ptr<SMSCL> sm1;
SMSCL sm;

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_pointcloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
    pcl_pointcloud->height = msg.height;
    pcl_pointcloud->width = msg.width;
    pcl_pointcloud->is_dense = false;
    double theta1 = M_PI/180*(pos-2048)/4096*360;
    *combine+=*lidar2base(pcl_pointcloud, theta1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(combine,0, 0, 255);
    {
        const std::lock_guard<std::mutex> lock(mex_viewer);

        pcl_viewer->updatePointCloud<pcl::PointXYZ>(combine, g, "rslidar");
    }

}


void pointCloudCallback1(const PointCloudMsg<pcl::PointXYZ>& msg)
{
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the message and process it in another thread is recommended*/
    RS_MSG << "1msg: " << msg.seq <<" msg.width: "<<msg.width <<" point cloud size: " << msg.point_cloud_ptr->size() << RS_REND;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_pointcloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
    pcl_pointcloud->height = msg.height;
    pcl_pointcloud->width = msg.width;
    pcl_pointcloud->is_dense = false;

    *combine+=*pcl_pointcloud;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(combine, 255, 0, 0);
    {
        const std::lock_guard<std::mutex> lock(mex_viewer);

        pcl_viewer->updatePointCloud<pcl::PointXYZ>(combine, r, "rslidar");
    }

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

void exceptionCallback1(const Error& code)
{
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the error message and process it in another thread is recommended*/
    RS_WARNING << "Error code1 : " << code.toString() << RS_REND;
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
    param.input_param.device_ip = "192.168.1.201";
    param.input_param.msop_port = 6691;   ///< Set the lidar msop port number, the default is 6699
    param.input_param.difop_port = 7781;  ///< Set the lidar difop port number, the default is 7788
    param.lidar_type = LidarType::RS16;   ///< Set the lidar type. Make sure this type is correct
    param.decoder_param.transform_param.yaw = 1.57;
    param.decoder_param.transform_param.x = 5.0;
    param.decoder_param.transform_param.y = 5.0;
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


    LidarDriver<pcl::PointXYZ> driver1;  ///< Declare the driver object

    RSDriverParam param1;                  ///< Create a parameter object
    param1.input_param.device_ip = "192.168.1.200";
    param1.input_param.msop_port = 6699;   ///< Set the lidar msop port number, the default is 6699
    param1.input_param.difop_port = 7788;  ///< Set the lidar difop port number, the default is 7788
    param1.lidar_type = LidarType::RS16;   ///< Set the lidar type. Make sure this type is correct
    param1.decoder_param.transform_param.x = -5.0;
    param1.decoder_param.transform_param.y = -5.0;
    param1.print();

    driver1.regExceptionCallback(exceptionCallback1);  ///< Register the exception callback function into the driver
    driver1.regRecvCallback(pointCloudCallback1);      ///< Register the point cloud callback function into the driver
    if (!driver1.init(param1))                         ///< Call the init function and pass the parameter
    {
        RS_ERROR << "Driver Initialize Error..." << RS_REND;
        return -1;
    }
    driver1.start();  ///< The driver thread will start
    RS_DEBUG << "1RoboSense Lidar-Driver Linux online demo start......" << RS_REND;


    pcl_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("RS LIDAR 16 Viewer");
    pcl_viewer->addCoordinateSystem();
//    int v1(0), v2(1);
//    pcl_viewer->createViewPort(0.0, 0.0, 1.0 / 2.0, 1.0/2.0, v1);
//    pcl_viewer->createViewPort(1.0 / 2.0, 1.0 / 2.0, 1.0, 1.0, v2);
    combine = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//    combine1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();;
    pcl_viewer->addPointCloud(combine, "rslidar");
//    pcl_viewer->addPointCloud(combine, "rslidar2");



//
//    while (true)
//    {
////        sleep(1);
//        show(viewer);
//    }
    while (!pcl_viewer->wasStopped())
    {
        {
            const std::lock_guard<std::mutex> lock(mex_viewer);
            pcl_viewer->spinOnce();
        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
//    t.join();
    return 0;


}