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


open3d::registration::RegistrationResult GeometryMethods::GetCorrespondencesNearestNeighbor(
        const open3d::geometry::PointCloud &source_cs,
        const open3d::geometry::PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation)
{
    using namespace open3d::registration;
    using namespace open3d::geometry;

    RegistrationResult result(transformation);
    if (max_correspondence_distance <= 0.0) {
        return result;
    }

    double error2 = 0.0;
    const KDTreeFlann target_kdtree(target);
    PointCloud source = source_cs;
    source.Transform(transformation);
#ifdef _OPENMP
#pragma omp parallel
    {
#endif
        double error2_private = 0.0;
        CorrespondenceSet correspondence_set_private;
#ifdef _OPENMP
#pragma omp for nowait
#endif
        for (int i = 0; i < (int)source.points_.size(); i++) {
            std::vector<int> indices(1);
            std::vector<double> dists(1);
            const auto &point = source.points_[i];
            if (target_kdtree.SearchHybrid(point, max_correspondence_distance,
                                           1, indices, dists) > 0) {
                error2_private += dists[0];
                correspondence_set_private.push_back(
                        Eigen::Vector2i(i, indices[0]));
            }
        }
#ifdef _OPENMP
#pragma omp critical
#endif
        {
            for (int i = 0; i < (int)correspondence_set_private.size(); i++) {
                result.correspondence_set_.push_back(
                        correspondence_set_private[i]);
            }
            error2 += error2_private;
        }
#ifdef _OPENMP
    }
#endif

    if (result.correspondence_set_.empty()) {
        result.fitness_ = 0.0;
        result.inlier_rmse_ = 0.0;
    } else {
        size_t corres_number = result.correspondence_set_.size();
        result.fitness_ = (double)corres_number / (double)source.points_.size();
        result.inlier_rmse_ = std::sqrt(error2 / (double)corres_number);
    }
    return result;
}
