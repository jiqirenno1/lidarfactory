//
// Created by ubuntu on 2020/11/4.
//

#ifndef LIDARFACTORY_PROCESSPOINTCLOUDS_H
#define LIDARFACTORY_PROCESSPOINTCLOUDS_H

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/organized_edge_detection.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdPtr;
class ProcessPointClouds {
public:
    ProcessPointClouds();
    ~ProcessPointClouds();
    PtCdPtr DownSampleCloud(PtCdPtr cloud, float res);
    PtCdPtr CropCloud(PtCdPtr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
    std::pair<PtCdPtr, PtCdPtr> SegmentPlane(PtCdPtr cloud, int maxIterations, float distance);
    std::pair<PtCdPtr, PtCdPtr> SegmentPlaneWithNormal(PtCdPtr cloud, int maxIterations, float distance);
    std::pair<PtCdPtr, PtCdPtr> SegmentCylinder(PtCdPtr cloud);
    std::pair<PtCdPtr, PtCdPtr> SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdPtr cloud);
    pcl::PointCloud<pcl::Normal>::Ptr GetNormals(PtCdPtr cloud);
    pcl::PolygonMesh GreedyTriangle(PtCdPtr cloud);
    pcl::PolygonMesh PoissonTriangle(PtCdPtr cloud);
    pcl::PolygonMesh MarchingCubeTriangle(PtCdPtr cloud);
    pcl::PolygonMesh CalConvexHull(PtCdPtr cloud);
    PtCdPtr BilateralFilter(PtCdPtr cloud);




};


#endif //LIDARFACTORY_PROCESSPOINTCLOUDS_H
