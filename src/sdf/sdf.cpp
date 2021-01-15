//
// Created by ubuntu on 2021/1/15.
//

#include "sdf.h"

void sdf::init(int m, float width, float height, float depth, Eigen::Vector3f &origin) {
    m_ = m;
    width_ = width;
    height_ = height;
    depth_ = depth;
    num_of_voxels_ = m*m*m;
    sdf_origin = origin;
    mc = std::make_shared<mypcl::MarchingCubesSDF>();

    voxel_coords = new Eigen::Vector3i[(m-2)*(m-2)*(m-2)];
    D = new float[num_of_voxels_];
    W = new float[num_of_voxels_];

    Eigen::Vector3i voxel_coordinates;
    int vox_ind = 0;
    for(int i=0;i<num_of_voxels_;i++)
    {
        D[i] = 100;
        W[i] = 0;
        get_voxel_coordinates(i, voxel_coordinates);
        if((voxel_coordinates(0)>0 && voxel_coordinates(0)<m-1) && (voxel_coordinates(1)>0 && voxel_coordinates(1)<m-1) && (voxel_coordinates(2)>0 && voxel_coordinates(2)<m-1))
        {
            voxel_coords[vox_ind] = voxel_coordinates;
            vox_ind++;
        }
    }


    mc->setIsoLevel(0.0);
    mc->setGridResolution(m, m, m);
    mc->setBBox(width, height, depth);
    mc->setGrid(D);
    mc->setW(W);
    mc->setVoxelCoordinates(voxel_coords);


}

void sdf::update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int nums = cloud->size();
    for(int i=0;i<nums;i++)
    {
        float x = cloud->points.at(i).x;
        float y = cloud->points.at(i).y;
        float z = cloud->points.at(i).z;
        if(isnan(x)||isnan(y)||isnan(z))
        {
            continue;
        }
        Eigen::Vector3f global0;
        global0 << x, y, z;
        Eigen::Vector3i voxel;
        get_voxel_coordinates(global0, voxel);

        int index = get_array_index(voxel);
        if(index==-1)
            continue;
        Eigen::Vector3f center;
        get_global_coordinates(voxel, center);

        std::vector<int> voxels(8);
        voxels[0] = index;
        voxels[1] = index+m_*m_;
        voxels[2] = voxels[1]+1;
        voxels[3] = voxels[0]+1;
        voxels[4] = voxels[0]+m_;
        voxels[5] = voxels[4]+m_*m_;
        voxels[6] = voxels[5]+1;
        voxels[7] = voxels[4]+1;

        for (int i = 0; i < 8; i++)
        {
            Eigen::Vector3f point = center;
            if(i	 & 0x4)
                point[1] = static_cast<float> (center[1] + height_ / float (m_));

            if(i & 0x2)
                point[2] = static_cast<float> (center[2] + depth_ / float (m_));

            if((i & 0x1) ^ ((i >> 1) & 0x1))
                point[0] = static_cast<float> (center[0] + width_ / float (m_));

            float dis = sqrt(x*x+y*y+z*z) - sqrt(point.x()*point.x()+point.y()*point.y()+point.z()*point.z());
            int index = voxels[i];
            if (index < 0 || index >= num_of_voxels_)
                continue;
            D[index] = (D[index]*W[index]+dis)/(W[index]+1);
            W[index] = W[index] + 1;
        }



    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr sdf::get_result() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    mc->performReconstruction(*cloud);
    return cloud;
}
