#include "manager.h"

int main()
{
    std::shared_ptr<Manager> demo_ptr = std::make_shared<Manager>();
    YAML::Node config, lidar_config;

    try
    {
        config = YAML::LoadFile("/home/ubuntu/CLionProjects/lidarfactory/config/config.yaml");
        lidar_config = yamlSubNodeAbort(config, "lidar");

    }
    catch (...)
    {
        RS_ERROR<<"Config file format wrong!"<<RS_REND;
        return -1;
    }
    demo_ptr->init(lidar_config);
    demo_ptr->start();

}