//
// Created by ubuntu on 2020/11/4.
//

#include "ProcessPointClouds.h"

ProcessPointClouds::ProcessPointClouds() {}

ProcessPointClouds::~ProcessPointClouds() {}

PtCdPtr ProcessPointClouds::DownSampleCloud(PtCdPtr cloud, float res) {
    pcl::VoxelGrid<PointT> vg;
    PtCdPtr downCloud(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(res, res, res);
    vg.filter(*downCloud);
    return downCloud;
}

PtCdPtr ProcessPointClouds::CropCloud(PtCdPtr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
    PtCdPtr regionCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud);
    region.filter(*regionCloud);
    return regionCloud;
}


PtCdPtr ProcessPointClouds::CropCloudZ(PtCdPtr cloud, float minZ, float maxZ) {
    PtCdPtr outCloud(new pcl::PointCloud<PointT>);
    size_t nums = cloud->size();
    for(size_t i=0;i<nums;i++)
    {
        float z=cloud->points[i].z;
        if(z>=minZ&&z<=maxZ)
        {
            outCloud->points.push_back(cloud->points[i]);
        }

    }
    return outCloud;
}

std::pair<PtCdPtr, PtCdPtr> ProcessPointClouds::SegmentPlane(PtCdPtr cloud, int maxIterations, float distance) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distance);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficient);
    std::vector<float> coeff = coefficient->values;
    //print coeff
//    for(auto &e:coeff)
//    {
//        std::cout<<"coeff: "<<e<<std::endl;
//
//    }


    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    std::pair<PtCdPtr, PtCdPtr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

std::pair<PtCdPtr, PtCdPtr>
ProcessPointClouds::SegmentPlaneWithNormal(PtCdPtr cloud, int maxIterations, float distance) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    cloud_normal = GetNormals(cloud);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distance);

    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normal);
    seg.segment(*inliers, *coefficient);
    std::vector<float> coeff = coefficient->values;
    for(auto &e:coeff)
    {
        std::cout<<"coeff: "<<e<<std::endl;

    }


    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    std::pair<PtCdPtr, PtCdPtr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

std::pair<PtCdPtr, PtCdPtr> ProcessPointClouds::SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdPtr cloud) {
    PtCdPtr planeCloud(new pcl::PointCloud<PointT>);
    PtCdPtr otherCloud(new pcl::PointCloud<PointT>);
    for(int index:inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*otherCloud);
    std::pair<PtCdPtr, PtCdPtr>segResult(planeCloud, otherCloud);

    return segResult;
}

std::pair<PtCdPtr, PtCdPtr> ProcessPointClouds::SegmentCylinder(PtCdPtr cloud) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    cloud_normal = GetNormals(cloud);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.1);

    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normal);
    seg.segment(*inliers, *coefficient);
    std::vector<float> coeff = coefficient->values;
    for(auto &e:coeff)
    {
        std::cout<<"coeff: "<<e<<std::endl;

    }


    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    std::pair<PtCdPtr, PtCdPtr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

pcl::PointCloud<pcl::Normal>::Ptr ProcessPointClouds::GetNormals(PtCdPtr cloud) {
    pcl::NormalEstimation<PointT, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius 1cm
    //n.setRadiusSearch(1);
    n.setKSearch(20);
    n.compute(*normals);
    return normals;
}

pcl::PolygonMesh ProcessPointClouds::GreedyTriangle(PtCdPtr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals = GetNormals(cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);   //* cloud_with_normals = cloud + normals
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
    pcl::PolygonMesh triangles;//存储最终三角化的网络模型

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.5);         //设置搜索半径radius，来确定三角化时k一邻近的球半径。

    // Set typical values for the parameters
    gp3.setMu (2.5);                     //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
    gp3.setMaximumNearestNeighbors (100);//设置样本点最多可以搜索的邻域数目100 。
    gp3.setMaximumSurfaceAngle(M_PI/4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
    gp3.setMinimumAngle(M_PI/18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
    gp3.setMaximumAngle(2*M_PI/3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
    gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。

    // Get result
    gp3.setInputCloud (cloud_with_normals);//设置输入点云为有向点云
    gp3.setSearchMethod (tree);           //设置搜索方式tree2
    gp3.reconstruct (triangles);           //重建提取三角化


    return triangles;
}



pcl::PolygonMesh ProcessPointClouds::PoissonTriangle(PtCdPtr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals =GetNormals(cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);   //* cloud_with_normals = cloud + normals
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);

    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
    pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
    pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
    pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
    //pn.setIndices();

    //设置搜索方法和输入点云
    pn.setSearchMethod(tree);
    pn.setInputCloud(cloud_with_normals);
    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh ;
    //执行重构
    pn.performReconstruction(mesh);

    return mesh;
}

pcl::PolygonMesh ProcessPointClouds::MarchingCubeTriangle(PtCdPtr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals =GetNormals(cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);   //* cloud_with_normals = cloud + normals
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);

    pcl::MarchingCubesHoppe<pcl::PointNormal>::Ptr mc(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
    pcl::PolygonMesh mesh;

    mc->setIsoLevel(0.0f);
    mc->setGridResolution(150,150,150);
    mc->setPercentageExtendGrid(0.0f);

    mc->setInputCloud(cloud_with_normals);
    mc->reconstruct(mesh);

    return mesh;

}


pcl::PolygonMesh  ProcessPointClouds::CalConvexHull(PtCdPtr cloud) {
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);
    hull.setComputeAreaVolume(true);
//    std::vector<pcl::Vertices> polygons;
//    pcl::PointCloud<PointT>::Ptr surface_hull(new pcl::PointCloud<PointT>);
    pcl::PolygonMesh mesh;
    hull.reconstruct(mesh);

    double convex_volume=hull.getTotalVolume();
    std::cout<<"convex volume: "<<convex_volume<<std::endl;

    return mesh;
}

PtCdPtr ProcessPointClouds::BilateralFilter(PtCdPtr cloud) {
    PtCdPtr outCloud(new pcl::PointCloud<PointT>);
    pcl::FastBilateralFilter<PointT> fbf;
    fbf.setInputCloud(cloud);
    fbf.setSigmaR(0.05f);
    fbf.setSigmaS(15.0f);
    fbf.filter(*outCloud);

    return outCloud;
}

pcl::PointCloud<pcl::PointNormal> ProcessPointClouds::Smoothing(PtCdPtr cloud) {

    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.5);

    //  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::RANDOM_UNIFORM_DENSITY);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::VOXEL_GRID_DILATION);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
    mls.setPointDensity ( 20*int (1));
//    mls.setUpsamplingRadius (1);
//    mls.setUpsamplingStepSize (0.5);
//    mls.setDilationIterations (2);
//    mls.setDilationVoxelSize (0.8f);

    mls.process(mls_points);
    return mls_points;
}

PtCdPtr ProcessPointClouds::PassThrough(PtCdPtr cloud, std::string axis, float min, float max) {
    PtCdPtr out(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min, max);
    pass.filter(*out);
    return out;
}

PtCdPtr ProcessPointClouds::RemovalOutlier(PtCdPtr cloud) {
    PtCdPtr res(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100); //近邻搜索点个数
    sor.setStddevMulThresh(1.0); //标准差倍数
    sor.setNegative(false); //保留未滤波点（内点）
    sor.filter(*res);
    return res;
}

std::vector<PtCdPtr> ProcessPointClouds::RegionGrowing(PtCdPtr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals =GetNormals(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize(300);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(50);
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0/180.0*M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::vector<PtCdPtr> cls;
    for(auto &indices:clusters)
    {
        PtCdPtr cluster(new pcl::PointCloud<PointT>);
        for(auto &indice:indices.indices)
        {
            cluster->points.push_back(cloud->points[indice]);
        }
        cls.push_back(cluster);

    }

//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer2"));
//    viewer->addPointCloud(colored_cloud);
    std::sort(cls.begin(), cls.end(), [](PtCdPtr a, PtCdPtr b){return a->size()>b->size();});
    return cls;

}

PtCdPtr ProcessPointClouds::EstimateBoundary(PtCdPtr cloud) {
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> boundEst;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals = GetNormals(cloud);
    boundEst.setInputCloud(cloud);
    boundEst.setInputNormals(normals);
    boundEst.setRadiusSearch(50);
    boundEst.setAngleThreshold(M_PI/4);
    boundEst.setSearchMethod(tree);
    boundEst.compute(boundaries);

    PtCdPtr out(new pcl::PointCloud<PointT>);
    for(size_t i=0;i<cloud->points.size();i++)
    {
        if(boundaries[i].boundary_point>0)
        {
            out->push_back(cloud->points[i]);
        }
    }

    return out;
}

PtCdPtr ProcessPointClouds::EstimateUpNet(PtCdPtr cloud) {
    PtCdPtr out(new pcl::PointCloud<PointT>);
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    float s = 2;
    std::vector<size_t> indexs;
    for(float i=minPt.x;i<=maxPt.x;i+=s)
    {
        for(float j=minPt.y;j<=maxPt.y;j+=s)
        {
            float maxZ = 0;
            size_t indexZ = 0;
            for(size_t k=0;k<cloud->size();k++)
            {
                float x = cloud->points[k].x;
                float y = cloud->points[k].y;
                if(x>=i&&x<i+s&&y>=j&&y<j+s)
                {
                    float z = cloud->points[k].z;
                    if(z>=maxZ)
                    {
                        maxZ = z;
                        indexZ = k;
                    }

                }
            }
            indexs.push_back(indexZ);
        }
    }

    for(auto & i:indexs)
    {
        out->points.push_back(cloud->points[i]);
    }
    return out;
}



