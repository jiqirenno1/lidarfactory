//
// Created by ubuntu on 2021/2/25.
//

#ifndef LIDARFACTORY_MANAGER_H
#define LIDARFACTORY_MANAGER_H

#include "MySMSCL.h"
#include "utils/yaml_reader.hpp"
#include "rs_driver/api/lidar_driver.h"
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace robosense::lidar;

class Manager {
public:
    Manager()=default;
    ~Manager();
    void init(const YAML::Node& config);
    void start();
    void exceptionCallback(const Error& code);
    void pointCloudCallback(const PointCloudMsg<pcl::PointXYZ>& msg);
private:
    std::shared_ptr<MySMSCL> sm_ptr_;
    std::shared_ptr<LidarDriver<pcl::PointXYZ>> driver_ptr_;
    std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer_;
    std::mutex mex_viewer_;


};


#endif //LIDARFACTORY_MANAGER_H
