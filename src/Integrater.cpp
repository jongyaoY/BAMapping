#include "Integrater.h"
#include "util/Parser.h"

using namespace BAMapping;

Integrater::Integrater()
{

}

void Integrater::integrateGraph(const Graph &graph, const char *config_file, const char* plyFile_name, const Mat4 Twc0)
{
    using namespace open3d;
    Parser config(config_file);

    int every_n_frames = config.getValue<int>("Integrater.every_n_frames");
    if(every_n_frames == 0)
        every_n_frames = 1;
    auto volume = createVolume(config);

    for(int i = 0; i < graph.nodes_.size(); i++)
    {
        if(i % every_n_frames != 0)
            continue;

        printf("\r integrating: %d node", i);
        fflush(stdout);

        auto node = graph.nodes_[i];
        Mat4 Tc0cn = node.pose_;
        Mat4 Twcn = Twc0 * Tc0cn;
        auto intrinsics = getIntrinsics(config);
        Mat4 extrinsics = Twcn.inverse();

        geometry::RGBDImage rgbd;
        bool success = createRGBDImage(config, node.depth_path_.c_str(), node.rgb_path_.c_str(), rgbd, false);
        if(success)
            volume->Integrate(rgbd,intrinsics,extrinsics);
    }

    io::WriteTriangleMesh(plyFile_name,*volume->ExtractTriangleMesh());
    visualization::DrawGeometries({volume->ExtractTriangleMesh()});

}

void Integrater::integrate(Parser config, std::string poseGraphName, const FrameVector frameVector,const char* plyFile_name)
{
    using namespace open3d;
    auto volume = createVolume(config);
    registration::PoseGraph global_poseGraph;
    io::ReadPoseGraph(poseGraphName,global_poseGraph);

    for(size_t fragment_id = 0; fragment_id < global_poseGraph.nodes_.size(); fragment_id++) //global_poseGraph.nodes_.size()-1
    {
        auto Twc0 = global_poseGraph.nodes_[fragment_id].pose_;
        integrateFragment(config,volume,fragment_id,frameVector,Twc0);//todo
    }
    io::WriteTriangleMesh(plyFile_name,*volume->ExtractTriangleMesh());
    visualization::DrawGeometries({volume->ExtractTriangleMesh()});
}



Integrater::Volume Integrater::createVolume(Parser config)
{
    using namespace open3d;

    double voxel_size = config.getValue<double>("Integrater.volume_size")/config.getValue<double>("Integrater.resolution");
    double sdf_trunc = config.getValue<double>("Integrater.sdf_trunc");
    bool color = config.getValue<bool>("Integrater.color");
    integration::TSDFVolumeColorType type;
    if(color)
        type = integration::TSDFVolumeColorType::RGB8;
    else
        type = integration::TSDFVolumeColorType::Gray32;

    Volume volume(new integration::ScalableTSDFVolume(voxel_size,sdf_trunc,type));

    return volume;
}

void Integrater::integrateFragment(Parser config, Integrater::Volume volume, const size_t fragment_id,
                                   const FrameVector frameVector, const Eigen::Matrix4d Twc0)
{
    using namespace open3d;


    size_t n_frame_per_fragment = config.getValue<int>("n_frames_per_fragment");
    size_t n_overlap = config.getValue<int>("n_overlap_frames");
    size_t start_node_id;
    size_t gap;

    if(fragment_id == 0)
    {
        start_node_id = 0;
        gap = 0;
    }
    else
    {
        start_node_id = n_overlap;
        gap = n_frame_per_fragment - n_overlap;
    }

    FrameVector local_frameVec;
    local_frameVec.insert(local_frameVec.end(),
            frameVector.begin()+fragment_id*gap,
                          std::min(frameVector.begin() + fragment_id*gap + n_frame_per_fragment,frameVector.end()));

    registration::PoseGraph fragment_poseGraph;
    io::ReadPoseGraph(Parser::poseGraphName(fragment_id),fragment_poseGraph);

    for(size_t node_id = start_node_id; node_id < fragment_poseGraph.nodes_.size(); node_id++)
    {
        if(node_id%2 != 0) //every two frames todo
            continue;

        auto node = fragment_poseGraph.nodes_[node_id];
        auto frame = local_frameVec[node_id];
        auto Tc0cn = node.pose_;
        auto pose = Twc0 * Tc0cn;
        geometry::RGBDImage rgbd;
        createRGBDImageFromFrame(frame,config,rgbd,false);


        int width = config.getValue<int>("Camera.width");
        int height = config.getValue<int>("Camera.height");
        double fx = config.getValue<double>("Camera.fx");
        double fy = config.getValue<double>("Camera.fy");
        double cx = config.getValue<double>("Camera.cx");
        double cy = config.getValue<double>("Camera.cy");

        camera::PinholeCameraIntrinsic intrinsic;
        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);

        volume->Integrate(rgbd,intrinsic,pose.inverse());

    }

}

open3d::camera::PinholeCameraIntrinsic Integrater::getIntrinsics(Parser config)
{
    int width = config.getValue<int>("Camera.width");
    int height = config.getValue<int>("Camera.height");
    double fx = config.getValue<double>("Camera.fx");
    double fy = config.getValue<double>("Camera.fy");
    double cx = config.getValue<double>("Camera.cx");
    double cy = config.getValue<double>("Camera.cy");

    open3d::camera::PinholeCameraIntrinsic intrinsic;
    intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
    return intrinsic;
}

bool Integrater::createRGBDImage(Parser config, const char *depth_file, const char *rgb_file, open3d::geometry::RGBDImage &rgbd, bool grayScale)
{
    using namespace open3d;

    geometry::Image depth;
    geometry::Image rgb;
    double depth_factor = config.getValue<double>("depth_factor");
    double depth_truncate = config.getValue<double>("depth_truncate");

    bool read = false;
    read = io::ReadImage(depth_file, depth);
    if(!read)
        return false;

    read = io::ReadImage(rgb_file,rgb);
    if(!read)
        return false;

    rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
            rgb, depth, depth_factor,
            depth_truncate, grayScale);

    return true;
}

bool Integrater::createRGBDImageFromFrame(const Frame frame, Parser config, open3d::geometry::RGBDImage &rgbd, bool useIRImg)
{
    using namespace open3d;
    geometry::Image depth;
    geometry::Image infraRed;
    geometry::Image rgb;
    double depth_factor = config.getValue<double>("depth_factor");
    double depth_truncate = config.getValue<double>("depth_truncate");

    bool read = false;
    read = io::ReadImage(frame.getDepthImagePath().c_str(), depth);
    if(!read)
        return false;
    if(useIRImg)
    {
        read = io::ReadImageFromPNG(frame.getInfraRedImagePath().c_str(),infraRed);
        if(!read)
            return false;
        rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
                infraRed, depth, depth_factor,
                depth_truncate, true);
    }
    else
    {
        read = io::ReadImage(frame.getRGBImagePath().c_str(), rgb);
        if(!read)
            return false;

        rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
                rgb, depth, depth_factor,
                depth_truncate, false);
    }

    return true;
}
