//
// Created by ubuntu on 2021/1/15.
//

#include "marching_cubes_sdf.h"

//////////////////////////////////////////////////////////////////////////////////////////////
mypcl::MarchingCubesSDF::MarchingCubesSDF ()
        : min_p_ (), max_p_ (),  iso_level_ ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
mypcl::MarchingCubesSDF::~MarchingCubesSDF ()
{
}

void
mypcl::MarchingCubesSDF::setBBox (float width, float height, float depth)
{
    //pcl::getMinMax3D (*input_, min_p_, max_p_);
    min_p_[0] = 0;
    min_p_[1] = 0;
    min_p_[2] = 0;
    min_p_[3] = 0;
    max_p_[0] = width;
    max_p_[1] = height;
    max_p_[2] = depth;
    max_p_[3] = 0;
    /*min_p_ -= (max_p_ - min_p_) * percentage_extend_grid_/2;
    max_p_ += (max_p_ - min_p_) * percentage_extend_grid_/2;

    Eigen::Vector4f bounding_box_size = max_p_ - min_p_;

    bounding_box_size = max_p_ - min_p_;
    PCL_DEBUG ("[pcl::MarchingCubesHoppe::getBoundingBox] Size of Bounding Box is [%f, %f, %f]\n",
               bounding_box_size.x (), bounding_box_size.y (), bounding_box_size.z ());
    double max_size =
        (std::max) ((std::max)(bounding_box_size.x (), bounding_box_size.y ()),
            bounding_box_size.z ());
    (void)max_size;
    // ????
    // data_size_ = static_cast<uint64_t> (max_size / leaf_size_);
    PCL_DEBUG ("[pcl::MarchingCubesHoppe::getBoundingBox] Lower left point is [%f, %f, %f]\n",
               min_p_.x (), min_p_.y (), min_p_.z ());
    PCL_DEBUG ("[pcl::MarchingCubesHoppe::getBoundingBox] Upper left point is [%f, %f, %f]\n",
               max_p_.x (), max_p_.y (), max_p_.z ());*/
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
mypcl::MarchingCubesSDF::interpolateEdge (Eigen::Vector3f &p1,
                                        Eigen::Vector3f &p2,
                                        float val_p1,
                                        float val_p2,
                                        Eigen::Vector3f &output)
{
    float mu = (iso_level_ - val_p1) / (val_p2-val_p1);
    output = p1 + mu * (p2 - p1);
}



//////////////////////////////////////////////////////////////////////////////////////////////
void
mypcl::MarchingCubesSDF::createSurface (const float (&leaf_node)[8],
                                      const Eigen::Vector3i &index_3d,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

    int cubeindex = 0;
    Eigen::Vector3f vertex_list[12];
    if (leaf_node[0] < iso_level_) cubeindex |= 1;
    if (leaf_node[1] < iso_level_) cubeindex |= 2;
    if (leaf_node[2] < iso_level_) cubeindex |= 4;
    if (leaf_node[3] < iso_level_) cubeindex |= 8;
    if (leaf_node[4] < iso_level_) cubeindex |= 16;
    if (leaf_node[5] < iso_level_) cubeindex |= 32;
    if (leaf_node[6] < iso_level_) cubeindex |= 64;
    if (leaf_node[7] < iso_level_) cubeindex |= 128;

    // Cube is entirely in/out of the surface
    if (edgeTable[cubeindex] == 0)
        return;

    //Eigen::Vector4f index_3df (index_3d[0], index_3d[1], index_3d[2], 0.0f);
    Eigen::Vector3f center;// TODO coeff wise product = min_p_ + Eigen::Vector4f (1.0f/res_x_, 1.0f/res_y_, 1.0f/res_z_) * index_3df * (max_p_ - min_p_);
    center[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * float (index_3d[0]) / float (res_x_);
    center[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * float (index_3d[1]) / float (res_y_);
    center[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * float (index_3d[2]) / float (res_z_);

    Eigen::Vector3f *p = new Eigen::Vector3f[8];
    for (int i = 0; i < 8; i++)
    {
        Eigen::Vector3f point = center;
        if(i	 & 0x4)
            point[1] = static_cast<float> (center[1] + (max_p_[1] - min_p_[1]) / float (res_y_));

        if(i & 0x2)
            point[2] = static_cast<float> (center[2] + (max_p_[2] - min_p_[2]) / float (res_z_));

        if((i & 0x1) ^ ((i >> 1) & 0x1))
            point[0] = static_cast<float> (center[0] + (max_p_[0] - min_p_[0]) / float (res_x_));

        p[i] = point;
    }


    // Find the vertices where the surface intersects the cube
    if (edgeTable[cubeindex] & 1)
        interpolateEdge (p[0], p[1], leaf_node[0], leaf_node[1], vertex_list[0]);
    if (edgeTable[cubeindex] & 2)
        interpolateEdge (p[1], p[2], leaf_node[1], leaf_node[2], vertex_list[1]);
    if (edgeTable[cubeindex] & 4)
        interpolateEdge (p[2], p[3], leaf_node[2], leaf_node[3], vertex_list[2]);
    if (edgeTable[cubeindex] & 8)
        interpolateEdge (p[3], p[0], leaf_node[3], leaf_node[0], vertex_list[3]);
    if (edgeTable[cubeindex] & 16)
        interpolateEdge (p[4], p[5], leaf_node[4], leaf_node[5], vertex_list[4]);
    if (edgeTable[cubeindex] & 32)
        interpolateEdge (p[5], p[6], leaf_node[5], leaf_node[6], vertex_list[5]);
    if (edgeTable[cubeindex] & 64)
        interpolateEdge (p[6], p[7], leaf_node[6], leaf_node[7], vertex_list[6]);
    if (edgeTable[cubeindex] & 128)
        interpolateEdge (p[7], p[4], leaf_node[7], leaf_node[4], vertex_list[7]);
    if (edgeTable[cubeindex] & 256)
        interpolateEdge (p[0], p[4], leaf_node[0], leaf_node[4], vertex_list[8]);
    if (edgeTable[cubeindex] & 512)
        interpolateEdge (p[1], p[5], leaf_node[1], leaf_node[5], vertex_list[9]);
    if (edgeTable[cubeindex] & 1024)
        interpolateEdge (p[2], p[6], leaf_node[2], leaf_node[6], vertex_list[10]);
    if (edgeTable[cubeindex] & 2048)
        interpolateEdge (p[3], p[7], leaf_node[3], leaf_node[7], vertex_list[11]);


    // Create the triangle


    for (int i = 0; triTable[cubeindex][i] != -1; i+=3)
    {
        pcl::PointXYZ p1,p2,p3;
        p1.x = vertex_list[triTable[cubeindex][i  ]][0];
        p1.y = vertex_list[triTable[cubeindex][i  ]][1];
        p1.z = vertex_list[triTable[cubeindex][i  ]][2];

        cloud->push_back (p1);

        p2.x = vertex_list[triTable[cubeindex][i+1]][0];
        p2.y = vertex_list[triTable[cubeindex][i+1]][1];
        p2.z = vertex_list[triTable[cubeindex][i+1]][2];
        cloud->push_back (p2);

        p3.x = vertex_list[triTable[cubeindex][i+2]][0];
        p3.y = vertex_list[triTable[cubeindex][i+2]][1];
        p3.z = vertex_list[triTable[cubeindex][i+2]][2];

        cloud->push_back (p3);
    }


}


//////////////////////////////////////////////////////////////////////////////////////////////
void
mypcl::MarchingCubesSDF::getNeighborList1D (float (&leaf)[8],
                                          Eigen::Vector3i &pos)
{
    //leaf = std::vector<float> (8, 0.0f);
    // Eigen::Vector3i pos = index3d;
    int g0 = pos[0]*zy_index_offset + pos[1]*res_z_ + pos[2];
    //pos = index3d + Eigen::Vector3i (1, 0, 0);
    int g1 = g0 + zy_index_offset;
    //pos = index3d + Eigen::Vector3i (1, 0, 1);
    int g2 = g1 + 1;
    //pos = index3d + Eigen::Vector3i (0, 0, 1);
    int g3 = g0 + 1;
    //pos = index3d + Eigen::Vector3i (0, 1, 0);
    int g4 = g0 + res_z_;
    //pos = index3d + Eigen::Vector3i (1, 1, 0);
    int g5 = g4 + zy_index_offset;
    //pos = index3d + Eigen::Vector3i (1, 1, 1);
    int g6 = g5+1;
    //pos = index3d + Eigen::Vector3i (0, 1, 1);
    int g7 = g4+1;
    if (W_[g0] > 0 && W_[g1] > 0 && W_[g2] > 0 && W_[g3] > 0 && W_[g4] > 0 && W_[g5] > 0 && W_[g6] > 0 && W_[g7] >0 ){
        leaf[0] = grid_[g0];
        leaf[1] = grid_[g1];
        leaf[2] = grid_[g2];
        leaf[3] = grid_[g3];
        leaf[4] = grid_[g4];
        leaf[5] = grid_[g5];
        leaf[6] = grid_[g6];
        leaf[7] = grid_[g7];
    }
    else{
        leaf[0] =  grid_[g0];
        leaf[1] =  grid_[g0];
        leaf[2] =  grid_[g0];
        leaf[3] =  grid_[g0];
        leaf[4] =  grid_[g0];
        leaf[5] =  grid_[g0];
        leaf[6] =  grid_[g0];
        leaf[7] =  grid_[g0];
    }
}



void
mypcl::MarchingCubesSDF::performReconstruction (pcl::PointCloud<pcl::PointXYZ> &points)
{
    if (!(iso_level_ >= 0 && iso_level_ < 1))
    {
        PCL_ERROR ("[pcl::%s::performReconstruction] Invalid iso level %f! Please use a number between 0 and 1.\n", getClassName ().c_str (), iso_level_);
        points.width = points.height = 0;
        points.points.clear ();
        return;
    }

    // Create grid
    //grid_ = std::vector<float> (res_x_*res_y_*res_z_, 0.0f);


    // Run the actual marching cubes algorithm, store it into a point cloud,
    // and copy the point cloud + connectivity into output
    points.clear ();

//    int np = omp_get_max_threads();
//    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> local(np);
//    global_array = new pcl::PointCloud<pcl::PointXYZ>::Ptr[np];

//#pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZ>());
        //localCloud = boost::make_shared<PointCloud<pcl::PointXYZ> >();
        //global_array[omp_get_thread_num()] = localCloud;
//#pragma omp for
        for (int idx=0;idx<number_of_voxels;idx++){
            float leaf_node[8];
            Eigen::Vector3i index_3d = voxel_coords_[idx];
            getNeighborList1D (leaf_node, index_3d);
            createSurface (leaf_node, index_3d, localCloud);
        }
        points = *localCloud;
    }

//concatenate local point clouds
//    for (int i=0; i<np; i++){
//        points += *global_array[i];
//    }


}

