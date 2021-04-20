//
// Created by ubuntu on 2021/2/25.
//

#ifndef LIDARFACTORY_MANAGER_H
#define LIDARFACTORY_MANAGER_H

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <unordered_set>
#include "SM45.h"
#include "utils/yaml_reader.hpp"
#include "rs_driver/api/lidar_driver.h"
#include "ProcessPointClouds.h"
#include "utils/Httplib.h"

using namespace robosense::lidar;

class Manager {
public:
    Manager()=default;
    ~Manager();
    void init(const YAML::Node& config);
    void start();
    void stop();
    void exceptionCallback(const Error& code);
    void pointCloudCallback(const PointCloudMsg<pcl::PointXYZ>& msg);
    void joinMap();
    int  getNumFiles();
    PtCdPtr lidar2base(const PtCdPtr cloud, double theta);
private:
    std::shared_ptr<SM45> sm_ptr_;
    std::shared_ptr<LidarDriver<pcl::PointXYZ>> driver_ptr_;
    std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer_;
    std::mutex mex_viewer_;
    std::string dir_;
    std::unordered_set<int> pos_lists_;
    bool save_flag;


};


#endif //LIDARFACTORY_MANAGER_H
