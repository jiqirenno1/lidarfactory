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
    //init motor
    std::string port;
    int baudrate;
    yamlRead<std::string>(config["motor"], "port", port, "/dev/ttyUSB0");
    yamlRead<int>(config["motor"], "baudrate", baudrate, 115200);
    yamlRead<std::string>(config["motor"], "dir", dir_, "/home/ubuntu/lidar/1/");

    sm_ptr_ = std::make_shared<MySMSCL>();
    sm_ptr_->init(port, baudrate);
    std::thread t(&MySMSCL::start, sm_ptr_);
    t.detach();
    //init datadir
    save_flag = true;
    if (boost::filesystem::is_directory(dir_)) {
        boost::filesystem::remove_all(dir_);
        boost::filesystem::create_directory(dir_);
    }
    else{
        boost::filesystem::create_directory(dir_);
    }

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
        if(getNumFiles()>10&&save_flag)
        {
            save_flag = false;
            joinMap();
            pcl_viewer_->spin();
        }
        if(!save_flag)
        {
            stop();
            cout<<"ending"<<endl;
            break;
        }

    }
}

void Manager::stop() {
    sm_ptr_->stop();
    driver_ptr_->stop();
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

    int pos = sm_ptr_->getPos();
    if(pos>0&&save_flag)
    {
        if(!pos_lists_.count(pos))
        {
            pos_lists_.insert(pos);
            std::cout<<"save pos: "<<std::to_string(pos)<<std::endl;
            cout<<"map1： "<<pcl_pointcloud->size()<<endl;
            //pcl::io::savePCDFile (dir_+std::to_string(pos)+".pcd", *pcl_pointcloud);
            pcl::io::savePCDFileBinary(dir_+std::to_string(pos)+".pcd", *pcl_pointcloud);
        }
    }


}

void Manager::joinMap() {
    std::string datapath = dir_;
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{datapath},
                                               boost::filesystem::directory_iterator{});

    std::sort(paths.begin(), paths.end());
    PtCdPtr combine(new pcl::PointCloud<PointT>);


    int lastP=0;
    for(auto &e:paths) {
        PtCdPtr cloud(new pcl::PointCloud<PointT>);

        std::cout << e << std::endl;
        const std::string &name = e.string();
        pcl::io::loadPCDFile(name, *cloud);
        auto pos = name.find_last_of('/');
        auto leaf = name.substr(pos + 1, 4);
        int p = atoi(leaf.c_str());
        if ((p - lastP) < 1) {
            lastP = p;
            continue;
        }
        lastP = p;
        std::cout << p << "\n";
        double theta1 = M_PI / 180 * (p - 2048) / 4096 * 360;
        *combine += *lidar2base(cloud, theta1);
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(combine, 0,255,0);
    pcl_viewer_->addPointCloud(combine, g, "out");

    cout<<"map.size: "<<combine->size()<<endl;

    pcl::io::savePCDFileBinary(dir_+"combine.pcd", *combine);
    cout<<"save map done!"<<endl;


}

int Manager::getNumFiles() {
    int cnt = std::count_if(
            boost::filesystem::directory_iterator(dir_),
            boost::filesystem::directory_iterator(),
            static_cast<bool(*)(const boost::filesystem::path&)>(boost::filesystem::is_regular_file) );
    return cnt;
}

PtCdPtr Manager::lidar2base(const PtCdPtr cloud, double theta) {
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

Manager::~Manager() {

}



