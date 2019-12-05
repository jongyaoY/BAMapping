#ifndef INTEGRATER_H
#define INTEGRATER_H

#include <stdio.h>

#include <iostream>

#include "util/Frame.h"
#include <Open3D/Open3D.h>

class Integrater
{
public:
    Integrater();
    void init(std::string strSettingPath);
    bool integrateFrame(const Frame frame);
    bool saveTSDF(const char* path);
    bool generateMesh(bool visualize = true);
private:
    struct TSDF_Param
    {
        float tsdf_size;
        float tsdf_res;
        double depth_factor;
        double depth_truncate;

    };
    TSDF_Param mTSDF_param;
    std::shared_ptr<open3d::integration::ScalableTSDFVolume> mVolume_ptr;
    open3d::camera::PinholeCameraIntrinsic mIntrinsic;

};

#endif // INTEGRATER_H
