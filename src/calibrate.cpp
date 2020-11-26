//
// Created by ubuntu on 2020/11/3.
//
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
pcl::PointCloud<PointT>::Ptr clicked_points_3d(new PointCloudT);
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
std::vector<PointT> v;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* args) {

    if (event.getKeySym() == "r" && event.keyDown()) {
        std::cout <<"repick!"<<std::endl;
        viewer->removePointCloud("clicked_points");
        clicked_points_3d->clear();
        v.clear();
    }
}



    void pp_callback_PointsSelect(const pcl::visualization::PointPickingEvent& event, void* args)
{
    if(event.getPointIndex()==-1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    clicked_points_3d->points.push_back(current_point);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(clicked_points_3d, 255, 0, 0);
    viewer->removePointCloud("clicked_points");
    viewer->addPointCloud(clicked_points_3d, red, "clicked_points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "clicked_points");
    std::cout <<"x,y,z: "<<current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

    v.push_back(current_point);

}

void WritePoints()
{
    std::ofstream out("../click_points.txt");
    std::cout<<"write file done! "<<std::endl;
    for(auto it=v.begin();it!=v.end();++it)
    {
        out<<(*it).x<<" "<<(*it).y<<" "<<(*it).z<<std::endl;
    }
}

void ReadPoints(std::string file, std::vector<Eigen::Vector3d> & pts)
{
    std::ifstream ifs;
    ifs.open(file.c_str());
    if(!ifs.is_open())
    {
        std::cerr<<"fail to open file: "<<file<<std::endl;
        return;
    }
    std::string pt_line;
    Eigen::Vector3d pt;
    while(std::getline(ifs, pt_line)&&!pt_line.empty())
    {
        std::istringstream PtData(pt_line);
        PtData>>pt.x()>>pt.y()>>pt.z();
        pts.push_back(pt);
    }
    ifs.close();
}


Eigen::Isometry3d GetTranformation(std::vector<Eigen::Vector3d>& src, std::vector<Eigen::Vector3d>& dst)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::Matrix4d A = Eigen::Matrix4d::Ones();
    for(int i=0;i<src.size();i++)
    {
        A.block(i,0,1,3)<<src[i].transpose();

    }

    Eigen::Matrix<double, 4, 3> b;
    for(int i=0;i<dst.size();i++)
    {
        b.block(i,0,1,3)<<dst[i].transpose();

    }
    Eigen::Matrix<double, 4, 3> x = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix3d rotation_matrix1;
    rotation_matrix1<<x.transpose().block(0,0,3,3);

    T.rotate(rotation_matrix1);
    Eigen::Vector3d t1;
    t1<<x.block(3,0,1,3).transpose();

    T.pretranslate(t1);

    return T;


}

int main()
{
    pcl::PointCloud<PointT>::Ptr cloud(new PointCloudT);



    std::string filename("/home/ubuntu/lidar/combine.pcd");

    if(pcl::io::loadPCDFile(filename, *cloud))
    {
        std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    }
    std::cout<<"* pointcloud size: "<<cloud->points.size()<<std::endl;

    viewer->addCoordinateSystem();
    viewer->addPointCloud(cloud, "calib");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "calib");
//    viewer->setCameraPosition(0,0,-5,-2,-1,0,0);
    viewer->registerPointPickingCallback(pp_callback_PointsSelect, (void*)&cloud);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&cloud);
    std::cout << "Shift+click to pick points, r for reset,  then press 'Q'..." << std::endl;
    viewer->spin();

    int vSize = v.size();
    std::cout<<"points size: "<<vSize<<std::endl;
    if(vSize==4)
    {
        WritePoints();
        std::vector<Eigen::Vector3d> pts;
        for(int i=0;i<4;i++)
        {
            Eigen::Vector3d pt(v[i].x, v[i].y, v[i].z);
            pts.push_back(pt);
        }
        Eigen::Isometry3d T = GetTranformation(pts, pts);
        std::cout<<"result: "<<T.matrix()<<std::endl;



    }


//    while(!viewer->wasStopped())
//    {
//        viewer->spinOnce();
//
//    }

    return 0;
}

