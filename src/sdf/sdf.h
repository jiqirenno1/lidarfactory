//
// Created by ubuntu on 2021/1/15.
//

#ifndef LIDARFACTORY_SDF_H
#define LIDARFACTORY_SDF_H

#include "marching_cubes_sdf.h"
class sdf {
public:
    sdf(){};
    ~sdf(){};
    void init(int m, float width, float height, float depth, Eigen::Vector3f& origin);
    void update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_result();

    inline void get_voxel_coordinates(int array_idx, Eigen::Vector3i& voxel_coordinates) const {
        voxel_coordinates(1) = (int) (array_idx % (m_*m_))/m_;
        voxel_coordinates(0) = (int) (array_idx/(m_*m_));
        voxel_coordinates(2) = (int) array_idx%m_;
    }

    inline void get_voxel_coordinates(Eigen::Vector3f& global_coordinates, Eigen::Vector3i& voxel_coordinates) const
    {
        voxel_coordinates(0) = (global_coordinates(0)-sdf_origin(0))*float(m_)/width_-0.5;
        voxel_coordinates(1) = (global_coordinates(1)-sdf_origin(1))*float(m_)/height_-0.5;
        voxel_coordinates(2) = (global_coordinates(2)-sdf_origin(2))*float(m_)/depth_-0.5;

    }

    inline int get_array_index(Eigen::Vector3i& voxel_coordinates) const{
        if (voxel_coordinates(0) < 0 || voxel_coordinates(1) < 0 || voxel_coordinates(2) < 0){
            return -1;
        }
        if (voxel_coordinates(0) >= m_ || voxel_coordinates(1) >= m_ || voxel_coordinates(2) >= m_){
            return -1;
        }
        int _idx = m_*m_*voxel_coordinates(0)+m_*voxel_coordinates(1)+voxel_coordinates(2);
        if (_idx < 0 || _idx >= num_of_voxels_){
            std::cout << "Error in get_array_index \n"<< voxel_coordinates << std::endl;
            _idx = -1;
        }

        return _idx;
    }

    inline void get_global_coordinates(Eigen::Vector3i& voxel, Eigen::Vector3f& global) const {
        global.x() = (voxel.x()+0.5)*width_/float(m_) + sdf_origin.x();
        global.y() = (voxel.y()+0.5)*height_/float(m_) + sdf_origin.y();
        global.z() = (voxel.z()+0.5)*depth_/float(m_) + sdf_origin.z();
    }
private:
    int m_;
    int num_of_voxels_;
    float width_, height_, depth_;
    float *D;
    float *W;
    Eigen::Vector3i *voxel_coords;
    Eigen::Vector3f sdf_origin;
    std::shared_ptr<mypcl::MarchingCubesSDF> mc;

};


#endif //LIDARFACTORY_SDF_H
