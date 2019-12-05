#include "Integrater.h"
#include "util/Parser.h"

Integrater::Integrater()
{

}
void Integrater::init(std::string strSettingPath)
{
    Parser parser;
    parser.load(strSettingPath);
    int width = parser.getValue<int>("Camera.width");
    int height = parser.getValue<int>("Camera.height");
    double fx = parser.getValue<double>("Camera.fx");
    double fy = parser.getValue<double>("Camera.fy");
    double cx = parser.getValue<double>("Camera.cx");
    double cy = parser.getValue<double>("Camera.cy");
    mIntrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
    mTSDF_param.tsdf_size = parser.getValue<float>("TSDF.size");

    mTSDF_param.tsdf_res = parser.getValue<float>("TSDF.resolution");
    mTSDF_param.depth_factor = parser.getValue<double>("TSDF.depth_factor");
    mTSDF_param.depth_truncate = parser.getValue<double>("TSDF.depth_truncate");

    mVolume_ptr.reset(
            new open3d::integration::ScalableTSDFVolume
            (mTSDF_param.tsdf_size/mTSDF_param.tsdf_res,
             0.01*mTSDF_param.tsdf_size,
             open3d::integration::TSDFVolumeColorType::RGB8)
            );



}

bool Integrater::integrateFrame(const Frame frame)
{
    using namespace open3d;
    geometry::Image color;
    geometry::Image depth;

    io::ReadImage(frame.getRGBImagePath().c_str(), color);
    io::ReadImage(frame.getDepthImagePath().c_str(), depth);
    auto rgbd = geometry::RGBDImage::CreateFromColorAndDepth(
            color, depth, mTSDF_param.depth_factor,
                          mTSDF_param.depth_truncate, false);

    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();

//    auto pose = frame.getConstTwc();
    auto pose = frame.getConstTcw();

    extrinsic.topLeftCorner(3,3)<<pose(0,0),pose(0,1),pose(0,2),
                                               pose(1,0),pose(1,1),pose(1,2),
                                               pose(2,0),pose(2,1),pose(2,2);
    extrinsic.topRightCorner(3,1)<<pose(0,3),pose(1,3),pose(2,3);
    mVolume_ptr->Integrate(*rgbd,
                     mIntrinsic,
                     extrinsic);
}

bool Integrater::saveTSDF(const char* path)
{
//    auto mesh = mVolume.ExtractTriangleMesh();
//                    io::WriteTriangleMesh("mesh_" + save_index_str + ".ply",
//                                          *mesh);
    return true;
}

bool Integrater::generateMesh(bool visualize)
{
    using namespace open3d;
    auto mesh = mVolume_ptr->ExtractTriangleMesh();
//    mesh->ComputeVertexNormals();
    io::WriteTriangleMesh("mesh.ply",*mesh);
    if(visualize)
        visualization::DrawGeometriesWithCustomAnimation(
            {mesh}, "Animation", 1920, 1080);
}
