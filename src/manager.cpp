//
// Created by ubuntu on 2021/2/25.
//

#include "manager.h"

void Manager::init(const YAML::Node &config) {
    pcl_viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("RSPointCloudViewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_viewer_->addPointCloud<pcl::PointXYZ>(pcl_pointcloud, "rslidar");

    driver_ptr_.reset(new LidarDriver<pcl::PointXYZ>);
    RSDriverParam driver_param;
    std::string lidar_type;
    YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
    yamlRead<std::string>(driver_config, "frame_id", driver_param.frame_id, "rslidar");
    yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
    driver_param.lidar_type = driver_param.strToLidarType(lidar_type);

    yamlRead<std::string>(driver_config, "device_ip", driver_param.input_param.device_ip, "192.168.1.200");
    yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port, 6699);
    yamlRead<uint16_t>(driver_config, "difop_port", driver_param.input_param.difop_port, 7788);

    driver_ptr_->regExceptionCallback(std::bind(&Manager::exceptionCallback, this, std::placeholders::_1));
    driver_ptr_->regRecvCallback(std::bind(&Manager::pointCloudCallback, this, std::placeholders::_1));

    if (!driver_ptr_->init(driver_param))
    {
        RS_ERROR << "Driver Initialize Error...." << RS_REND;
        exit(-1);
    }

    std::string port;
    int baudrate;
    yamlRead<std::string>(config["motor"], "port", port, "/dev/ttyUSB0");
    yamlRead<int>(config["motor"], "baudrate", baudrate, 115200);

    sm_ptr_ = std::make_shared<MySMSCL>();
    sm_ptr_->init(port, baudrate);
    std::thread t(&MySMSCL::start, sm_ptr_);
    t.detach();

}
void Manager::start() {
    driver_ptr_->start();

    while (!pcl_viewer_->wasStopped())
    {
        {
            const std::lock_guard<std::mutex> lock(mex_viewer_);
            pcl_viewer_->spinOnce();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


}

void Manager::exceptionCallback(const Error &code) {
    RS_WARNING << code.toString() << RS_REND;
}

void Manager::pointCloudCallback(const PointCloudMsg<pcl::PointXYZ> &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_pointcloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
    pcl_pointcloud->height = msg.height;
    pcl_pointcloud->width = msg.width;
    pcl_pointcloud->is_dense = false;

    {
        const std::lock_guard<std::mutex> lock(mex_viewer_);
        pcl_viewer_->updatePointCloud<pcl::PointXYZ>(pcl_pointcloud, "rslidar");
    }


}

Manager::~Manager() {

}
