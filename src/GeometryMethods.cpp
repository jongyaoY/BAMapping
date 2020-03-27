//
// Created by jojo on 07.01.20.
//

#include "GeometryMethods.h"

using namespace BAMapping;
bool GeometryMethods::createRGBDImageFromNode(const Graph::Node &node, double depth_factor,double depth_truncate, open3d::geometry::RGBDImage &rgbd, bool useIRImg)
{
    using namespace open3d;
    geometry::Image depth;
    geometry::Image infraRed;
    geometry::Image rgb;

    bool read = false;
    read = io::ReadImage(node.depth_path_.c_str(), depth);
    if(!read)
        return false;
    if(useIRImg)
    {
        read = io::ReadImageFromPNG(node.ir_path_.c_str(),infraRed);
        if(!read)
            return false;
        rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
                infraRed, depth, depth_factor,
                depth_truncate, true);
    }
    else
    {
        read = io::ReadImage(node.rgb_path_.c_str(), rgb);
        if(!read)
            return false;

        rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
                rgb, depth, depth_factor,
                depth_truncate, false);
    }

    return true;
}

bool GeometryMethods::createPointCloundFromNodes(const std::vector<Graph::Node> nodes,
        Parser config,
        std::shared_ptr<open3d::geometry::PointCloud> &pcd,
        bool color)
{
    using namespace open3d;
    if(nodes.empty())
        return false;
    double voxel_size = config.getValue<double>("Integrater.volume_size")/config.getValue<double>("Integrater.resolution");
    double sdf_trunc = config.getValue<double>("Integrater.sdf_trunc");
    double depth_factor = config.getValue<double>("Integrater.depth_factor");
    double depth_truncate = config.getValue<double>("Integrater.depth_truncate");
    integration::TSDFVolumeColorType type;
    if(color)
    {
        type = integration::TSDFVolumeColorType::RGB8;
    }
    else
    {
        type = integration::TSDFVolumeColorType::Gray32;
    }
    integration::ScalableTSDFVolume volume(voxel_size,sdf_trunc,
                                           type);
    auto Twc0 = nodes[0].pose_; //set frame 0 as base Twc;

    for(auto node : nodes)
    {
        open3d::geometry::RGBDImage rgbd;
        bool success = createRGBDImageFromNode(node,depth_factor,depth_truncate,rgbd,!color);
        if(!success)
            break;

        int width = config.getValue<int>("Camera.width");
        int height = config.getValue<int>("Camera.height");
        double fx = config.getValue<double>("Camera.fx");
        double fy = config.getValue<double>("Camera.fy");
        double cx = config.getValue<double>("Camera.cx");
        double cy = config.getValue<double>("Camera.cy");

        camera::PinholeCameraIntrinsic intrinsic;

        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
        auto extrinsic = node.pose_.inverse() * Twc0;

        volume.Integrate(rgbd,intrinsic,extrinsic);
    }
    pcd = volume.ExtractPointCloud();
    if(pcd->points_.empty())
        return false;
    else
        return true;
}



bool GeometryMethods::createPointCloudFromNode(
        const Graph::Node node,
        Parser config,
        std::shared_ptr<open3d::geometry::PointCloud> &pcd,
        bool color)
{
    using namespace open3d;
    int width = config.getValue<int>("Camera.width");
    int height = config.getValue<int>("Camera.height");
    double fx = config.getValue<double>("Camera.fx");
    double fy = config.getValue<double>("Camera.fy");
    double cx = config.getValue<double>("Camera.cx");
    double cy = config.getValue<double>("Camera.cy");
    double depth_factor = config.getValue<double>("Integrater.depth_factor");
    double depth_truncate = config.getValue<double>("Integrater.depth_truncate");
    camera::PinholeCameraIntrinsic intrinsic;

    intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);

    geometry::RGBDImage rgbd;
    bool success = createRGBDImageFromNode(node,depth_factor,depth_truncate,rgbd,!color);
    if(success)
    {
        pcd = geometry::PointCloud::CreateFromRGBDImage(rgbd,intrinsic);
//        visualization::DrawGeometries({pcd});
        return true;
    }
    else
    {
        return false;
    }
}

bool GeometryMethods::createPointCloudFromNode(
        const Graph::Node node,
        const Vec4 &intrisics,
        const size_t width,
        const size_t height,
        std::shared_ptr<open3d::geometry::PointCloud> &pcd,
        bool color)
{
    using namespace open3d;

    double fx = intrisics[0];
    double fy = intrisics[1];
    double cx = intrisics[2];
    double cy = intrisics[3];

    camera::PinholeCameraIntrinsic intrinsic;

    intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);

    geometry::RGBDImage rgbd;
    bool success = createRGBDImageFromNode(node,5000.0,4.0,rgbd,!color);
    if(success)
    {
        pcd = geometry::PointCloud::CreateFromRGBDImage(rgbd,intrinsic);
//        visualization::DrawGeometries({pcd});
        return true;
    }
    else
    {
        return false;
    }
}
//bool GeometryMethods::createPointCloundFromFrames(const FrameVector frameVector, Parser config,
//                                                  std::shared_ptr<open3d::geometry::PointCloud> &pcd, bool color)
//{
//    using namespace open3d;
//    if(frameVector.empty())
//        return false;
//
//    double voxel_size = config.getValue<double>("volume_size")/config.getValue<double>("resolution");
//    double sdf_trunc = config.getValue<double>("sdf_trunc");
//    integration::TSDFVolumeColorType type;
//    if(color)
//    {
//        type = integration::TSDFVolumeColorType::RGB8;
//    }
//    else
//    {
//        type = integration::TSDFVolumeColorType::Gray32;
//    }
//    integration::ScalableTSDFVolume volume(voxel_size,sdf_trunc,
//                                           type);
//    auto Twc0 = frameVector[0].getConstTwc(); //set frame 0 as base
//    for(auto frame : frameVector)
//    {
//        open3d::geometry::RGBDImage rgbd;
//        bool success = createRGBDImageFromFrame(frame,config,rgbd,!color);
//        if(!success)
//            break;
//
//        int width = config.getValue<int>("Camera.width");
//        int height = config.getValue<int>("Camera.height");
//        double fx = config.getValue<double>("Camera.fx");
//        double fy = config.getValue<double>("Camera.fy");
//        double cx = config.getValue<double>("Camera.cx");
//        double cy = config.getValue<double>("Camera.cy");
//
//        camera::PinholeCameraIntrinsic intrinsic;
//
//        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
//        auto extrinsic = frame.getConstTcw() * Twc0;
//
//        volume.Integrate(rgbd,intrinsic,extrinsic);
//    }
//    pcd = volume.ExtractPointCloud();
//    if(pcd->points_.empty())
//        return false;
//    else
//        return true;
//}
