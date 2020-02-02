#ifndef INTEGRATER_H
#define INTEGRATER_H

#include <stdio.h>

#include <iostream>

#include "Frame.h"
#include "Graph.h"
#include "util/Parser.h"
#include "Open3D/Open3D.h"

namespace BAMapping
{
    class Integrater
    {
    public:
        typedef std::shared_ptr<open3d::integration::ScalableTSDFVolume> Volume;
        Integrater();
        static Volume createVolume(const Parser config);
        static void integrateGraph(const Graph& graph, const char* config_file, const char* plyFile_name, const Mat4 Twc0 = Mat4::Identity(), const bool visualize = false);
        static open3d::camera::PinholeCameraIntrinsic getIntrinsics(Parser config);
        static bool createRGBDImage(Parser config, const char* depth_file, const char* rgb_file, open3d::geometry::RGBDImage &rgbd, bool grayScale);
        static void integrate(const Parser config, const std::string poseGraphName, const FrameVector frameVector, const char* plyFile_name);
        static void integrateFragment(const Parser config, Volume volume, const size_t fragment_id, const FrameVector frameVector, const Eigen::Matrix4d Twc0);
        static bool createRGBDImageFromFrame(const Frame frame, Parser config, open3d::geometry::RGBDImage &rgbd, bool useIRImg);

    };

}

#endif // INTEGRATER_H
