#ifndef INTEGRATER_H
#define INTEGRATER_H

#include <stdio.h>

#include <iostream>
#include "pcl/io/pcd_io.h"
//#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"

#include "opencv4/opencv2/opencv.hpp"

#include "cpu_tsdf/tsdf_volume_octree.h"
#include "cpu_tsdf/marching_cubes_tsdf_octree.h"

#include "util/Frame.h"
class Integrater
{
public:
    Integrater();
    void init(std::string strSettingPath);
    bool integrateFrame(const Frame frame);
    void generatePointCloud(const char* rgbImg_path,const char* DepthImg_path,pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud);
    bool saveTSDF(const char* path);
    bool generateMesh(bool visualize = true);
private:
    struct TSDF_Param
    {
        float tsdf_size;
        float tsdf_res;
        float image_width;
        float image_height;
        float fx;
        float fy;
        float cx;
        float cy;
        int num_random_splits;
        float min_sensor_dist;
        float max_sensor_dist;
        bool integrate_color;
        float trunc_dist_pos;
        float trunc_dist_neg;
    };
    float mDepthFactor;  //For some datasets (e.g. TUM) the depthmap values are scaled.
    TSDF_Param mTSDF_param;
    cpu_tsdf::TSDFVolumeOctree::Ptr mTSDF;
};

#endif // INTEGRATER_H
