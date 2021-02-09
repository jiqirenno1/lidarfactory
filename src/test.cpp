//
// Created by ubuntu on 2020/11/4.
//
/*
#include <pcl/common/common.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "ProcessPointClouds.h"
#include <cmath>
#include <pcl/common/transforms.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));

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
    std::string file = "/home/ubuntu/lidar/combine.pcd";
    std::string file1 = "/home/ubuntu/lidar/sml.pcd";
    PtCdPtr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(file, *cloud);
//    std::cout<<"mesh: "<<cloud->height<<"\n";

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addCoordinateSystem();
    auto proI = new ProcessPointClouds();

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> rgb(cloud, "z");

    //PtCdPtr cloudP = proI->PassThrough(cloud, "x",8.0, 20.0);
    //PtCdPtr cloudP = cloud;
    PtCdPtr cloudP = proI->DownSampleCloud(cloud, 0.3);
    std::cout<<"before : "<<cloud->size()<<"\n";
    PtCdPtr res = proI->RemovalOutlier(cloudP);
    std::cout<<"after : "<<res->size()<<"\n";

    std::pair<PtCdPtr, PtCdPtr> result = proI->SegmentPlane(res, 50, 0.1);

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloudP, 0, 255, 0);
//    viewer->addPointCloud(cloudP, r, "cloudall");

//    PtCdPtr net = proI->EstimateUpNet(cloudP);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> g(net, 255, 0, 0);
//    viewer->addPointCloud(net,  g, "net");
//
    pcl::PointCloud<pcl::Normal>::Ptr nn = proI->GetNormals(cloudP);
//    PtCdPtr plane(new pcl::PointCloud<PointT>);
//    for(size_t i=0;i<nn->size();i++)
//    {
//        if(nn->points[i].curvature<=0.1)
//        {
//            plane->points.push_back(cloudP->points[i]);//        }
//    }
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> p(plane, 0, 255, 255);
//    viewer->addPointCloud(plane, p, "pplane");

//    std::vector<PtCdPtr> cls = proI->RegionGrowing(cloudP);
//    //viewer->addPointCloudNormals<PointT, pcl::Normal>(cloudP, nn, 2, 0.3); //每2个点显示一个及每个法线的长度0.3
//    for(auto & cl:cls)
//    {
//        std::cout<<"cls: "<<cl->size()<<"\n";
//    }
//
//    viewer->addPointCloud(cls[0],  "plane");
//    //pcl::io::savePCDFile ("/home/ubuntu/lidar/plane.pcd", *res);
//
//    PtCdPtr bound = proI->EstimateBoundary(cls[0]);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> b(bound, 255, 0, 0);
//    viewer->addPointCloud(bound, b, "boundry");

//    pcl::PointCloud<pcl::PointNormal> outCloud = proI->Smoothing(result.first);
//    std::cout<<"size 2 : "<<outCloud.size()<<"\n";
//
//    PtCdPtr sm(new pcl::PointCloud<PointT>);
//    size_t ss = outCloud.size();
//    sm->resize(ss);
//    for(size_t i=0;i<ss;i++)
//    {
//        sm->points[i].x = outCloud.points[i].x;
//        sm->points[i].y = outCloud.points[i].y;
//        sm->points[i].z = outCloud.points[i].z;
//    }
//    std::cout<<"size 3 : "<<sm->size()<<"\n";

//    viewer->addPointCloud(cloudP, r, "cloudall");

//    PtCdPtr smdown = proI->DownSampleCloud(sm, 0.3);
    pcl::PolygonMesh mesh=proI->GreedyTriangle(cloudP);
    viewer->addPolygonMesh(mesh, "mesh");
    std::cout<<"mesh: "<<mesh.polygons.size()<<"\n";
      viewer->spin();

}
//
int mainloop()
{
    viewer->setFullScreen(false);
    std::string file0 = "/home/ubuntu/lidar/poshe1/6pos2048.pcd";
    PtCdPtr cloud1(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(file0, *cloud1);

    std::string datapath = "/home/ubuntu/lidar/poshe1";
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{datapath},
                                               boost::filesystem::directory_iterator{});

    std::sort(paths.begin(), paths.end());
    PtCdPtr combine(new pcl::PointCloud<PointT>);
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("pcd viewer"));

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    viewer->setCameraPosition(-484.391530,91.717888,392.964559,0.608781,0.033757,0.792620);
    viewer->addCoordinateSystem();
    int lastP=0;
    for(auto &e:paths)
    {
        PtCdPtr cloud(new pcl::PointCloud<PointT>);

        std::cout<<e<<std::endl;
        const std::string& name = e.string();
        pcl::io::loadPCDFile(name, *cloud);
        auto pos = name.find_last_of('/');
        auto leaf = name.substr(pos+5, 4);
        int p = atoi(leaf.c_str());
        if((p-lastP)<1)
        {
            lastP = p;
            continue;
        }
        lastP = p;
        std::cout<<p<<"\n";
        double theta1 = M_PI/180*(p-2048)/4096*360;
        *combine+=*lidar2base(cloud, theta1);
        //std::cout<<"then combine size: "<<combine->size()<<std::endl;

        viewer->removePointCloud("cloud");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> r(combine,0, 255, 0);
        viewer->addPointCloud(combine, r, "cloud");
        viewer->spinOnce();
        if(p<2080)
        {
            sleep(1);

        }


    }
    ProcessPointClouds *proI =new ProcessPointClouds();
    PtCdPtr combine1 = proI->DownSampleCloud(combine, 0.3);
    combine = proI->DownSampleCloud(combine, 0.6);
    viewer->removePointCloud("cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(combine,0, 255, 0);
    viewer->addPointCloud(combine, green, "cloud");

    std::vector<PtCdPtr> cls = proI->RegionGrowing(combine);
    viewer->addPointCloud(cls[0],  "plane");
    PtCdPtr bound = proI->EstimateBoundary(cls[0]);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> b(bound, 255, 0, 0);
    viewer->addPointCloud(bound, b, "boundry");
    viewer->spinOnce();
    viewer->saveScreenshot("shot.jpg");
    sleep(5);

    //std::cout<<"then combine size: "<<combine->size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud1,0, 255, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> b(combine,0, 0,255);
//    viewer->addPointCloud(cloud1, r, "cloud1");
//    viewer->addPointCloud(combine, b, "cloud");

    //pcl::io::savePCDFile ("/home/ubuntu/lidar/combine0.pcd", *combine);
    pcl::PolygonMesh mesh= proI->GreedyTriangle(combine1);
    std::cout<<"mesh: "<<mesh.polygons.size()<<"\n";
    viewer->removeAllPointClouds();
    viewer->addPolygonMesh(mesh, "mesh");


    viewer->spin();


}
*/

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

#include "ProcessPointClouds.h"
typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    // Load input file
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile("/home/ubuntu/lidar/pp-p1.pcd", *cloud) == -1)
    {
        cout << "could not load the ile..." << endl;
    }

    pcl::PointCloud<PointT>::Ptr cloud_part(new pcl::PointCloud<PointT>);
    ProcessPointClouds *ppc = new ProcessPointClouds();
//    cloud_part = ppc->CropCloudZ(cloud, 5, 60);
//    pcl::io::savePCDFile ("/home/ubuntu/lidar/pp-p1.pcd", *cloud_part);


    std::cout << "Orginal points number: " << cloud->points.size() << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 1.下采样，同时保持点云形状特征
    pcl::VoxelGrid<PointT> downSampled;				// 下采样对象
    downSampled.setInputCloud(cloud);
    downSampled.setLeafSize(0.6f, 0.6f, 0.6f);	// 栅格叶的尺寸
    downSampled.filter(*cloud_downSampled);
    std::cout << "downsample points number: " << cloud_downSampled->points.size() << std::endl;
//    cloud_downSampled = cloud;

    // 2.统计滤波
    pcl::StatisticalOutlierRemoval<PointT> sor;		// 滤波对象
    sor.setInputCloud(cloud_downSampled);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);					// 设置判定为离群点的阈值
    sor.filter(*cloud_filtered);
//    viewer->addPointCloud(cloud_filtered, "dowm");
    std::cout << "filter points number: " << cloud_filtered->points.size() << std::endl;

    cloud_part = ppc->EstimateUpNet(cloud_filtered);
    cloud_filtered = cloud_part;
    std::cout << "filter points number: " << cloud_filtered->points.size() << std::endl;
    viewer->addPointCloud(cloud_filtered, "dowm");

    // 3.对点云重采样,进行平滑
    pcl::search::KdTree<PointT>::Ptr treeSampling(new pcl::search::KdTree<PointT>); // 创建用于最近邻搜索的KD-Tree
    pcl::MovingLeastSquares<PointT, PointT> mls;	// 定义最小二乘实现的对象mls
    mls.setComputeNormals(false);					// 设置在最小二乘计算中是否需要存储计算的法线
    mls.setInputCloud(cloud_filtered);				// 设置待处理点云
    mls.setPolynomialOrder(2);						// 拟合2阶多项式拟合
    //mls.setPolynomialFit(false);					// 设置为false可以 加速 smooth
    mls.setSearchMethod(treeSampling);				// 设置KD-Tree作为搜索方法
    mls.setSearchRadius(2.5);						// 单位m.设置用于拟合的K近邻半径
    mls.process(*cloud_smoothed);					// 输出
    pcl::io::savePCDFile ("/home/ubuntu/lidar/pp-m.pcd", *cloud_smoothed);
//    std::cout<<mls.getMLSResults().size()<<std::endl;
//    /**< \brief The polynomial coefficients Example: z = c_vec[0] + c_vec[1]*v + c_vec[2]*v^2 + c_vec[3]*u + c_vec[4]*u*v + c_vec[5]*u^2 */
//    std::cout<<cloud_smoothed->points[0]<<std::endl;
//    float u = cloud_smoothed->points[0].x;
//    float v = cloud_smoothed->points[0].y;
//    float z = cloud_smoothed->points[0].z;
//    Eigen::VectorXd c_vec= mls.getMLSResults()[0].c_vec;
//    float z1 = c_vec[0] + c_vec[1]*v + c_vec[2]*v*v + c_vec[3]*u + c_vec[4]*u*v + c_vec[5]*u*u;
//    std::cout<<z1<<std::endl;
//    std::cout<<mls.getMLSResults()[0].c_vec<<std::endl;
//    std::cout<<mls.getMLSResults()[1].c_vec<<std::endl;
    //cloud_smoothed = cloud_filtered;
    std::cout << "smooth points number: " << cloud_smoothed->points.size() << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> r(cloud_smoothed, 0, 255, 0);
    viewer->addPointCloud(cloud_smoothed, r, "dowmsam");
    viewer->spinOnce();

    // 4.法线估计
    pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;                    // 创建法线估计的对象
    normalEstimation.setInputCloud(cloud_smoothed);                                 // 输入点云
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);         // 创建用于最近邻搜索的KD-Tree
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);    // 定义输出的点云法线

    // K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
    normalEstimation.setKSearch(20);                    // 使用当前点周围最近的10个点
    //normalEstimation.setRadiusSearch(0.03);           // 对于每一个点都用半径为3cm的近邻搜索方式

    normalEstimation.compute(*normals); 				// 计算法线

    // 5.将点云位姿、颜色、法线信息连接到一起
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);

    // 6.贪心投影三角化

    //定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // 三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
    pcl::PolygonMesh triangles;									// 存储最终三角化的网络模型

    // 设置三角化参数
    gp3.setSearchRadius(2.8);				// 设置搜索时的半径，也就是KNN的球半径
    gp3.setMu(7);							// 设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
    gp3.setMaximumNearestNeighbors(200);    // 设置样本点最多可搜索的邻域个数，典型值是50-100

    gp3.setMinimumAngle(M_PI / 18);			// 设置三角化后得到的三角形内角的最小的角度为10°
    gp3.setMaximumAngle(2 * M_PI / 3);		// 设置三角化后得到的三角形内角的最大角度为120°

    gp3.setMaximumSurfaceAngle(M_PI / 4);	// 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
    gp3.setNormalConsistency(false);		// 设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

    gp3.setInputCloud(cloud_with_normals);  // 设置输入点云为有向点云
    gp3.setSearchMethod(tree2);				// 设置搜索方式
    gp3.reconstruct(triangles);				// 重建提取三角化

    // 7.显示网格化结果

    //viewer->setBackgroundColor(0, 0, 0);  		// 设置背景
//    viewer->addPolygonMesh(triangles, "mesh");  // 网格化点云添加到视窗
    viewer->spin();

//    while (!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
////       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
    return 1;
}

