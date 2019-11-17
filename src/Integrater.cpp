#include "Integrater.h"

Integrater::Integrater()
{
    mDepthFactor = 1/5000.;
}
void Integrater::init(std::string strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mTSDF_param.fx = fSettings["Camera.fx"];
    mTSDF_param.fy = fSettings["Camera.fy"];
    mTSDF_param.cx = fSettings["Camera.cx"];
    mTSDF_param.cy = fSettings["Camera.cy"];

    mTSDF_param.tsdf_size = fSettings["TSDF.size"];
    mTSDF_param.tsdf_res = fSettings["TSDF.resolution"];
    mTSDF_param.trunc_dist_pos = fSettings["TSDF.truncated_distance_pos"];
    mTSDF_param.trunc_dist_neg = fSettings["TSDF.truncated_distance_neg"];
    mTSDF_param.min_sensor_dist = fSettings["TSDF.min_sensor_dist"];
    mTSDF_param.max_sensor_dist = fSettings["TSDF.max_sensor_dist"];
    mTSDF_param.num_random_splits = fSettings["TSDF.num_random_splits"];
    mTSDF_param.image_width = fSettings["TSDF.image_width"];
    mTSDF_param.image_height = fSettings["TSDF.image_height"];
    mTSDF_param.integrate_color = true;//fSettings["TSDF.integrate_color"];

    mTSDF.reset (new cpu_tsdf::TSDFVolumeOctree);
    mTSDF->setGridSize(mTSDF_param.tsdf_size,mTSDF_param.tsdf_size,mTSDF_param.tsdf_size);
    mTSDF->setResolution(mTSDF_param.tsdf_res,mTSDF_param.tsdf_res,mTSDF_param.tsdf_res);
    mTSDF->setImageSize (mTSDF_param.image_width, mTSDF_param.image_height);
    mTSDF->setCameraIntrinsics (mTSDF_param.fx, mTSDF_param.fy, mTSDF_param.cx, mTSDF_param.cy);
    mTSDF->setNumRandomSplts (mTSDF_param.num_random_splits);
    mTSDF->setSensorDistanceBounds (mTSDF_param.min_sensor_dist, mTSDF_param.max_sensor_dist);
    mTSDF->setIntegrateColor (mTSDF_param.integrate_color);
    mTSDF->setDepthTruncationLimits (mTSDF_param.trunc_dist_pos, mTSDF_param.trunc_dist_neg);
    mTSDF->reset ();
}

bool Integrater::integrateFrame(const Frame frame)
{
    using namespace cpu_tsdf;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    generatePointCloud(frame.getRGBImagePath().c_str(),frame.getDepthImagePath().c_str(),cloud);
    Eigen::Affine3f pose;
    Eigen::Matrix4f Twc;
    pose.matrix() = frame.getConstTwc();
    mTSDF->integrateCloud(*cloud,pcl::PointCloud<pcl::PointNormal>(),pose.inverse().cast<double>());
}
void Integrater::generatePointCloud(const char* rgbImg_path,const char* DepthImg_path,pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
{
    cv::Mat img;
    cv::Mat depth;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    img = cv::imread(rgbImg_path,cv::IMREAD_COLOR);
    depth = cv::imread(DepthImg_path,cv::IMREAD_UNCHANGED);
    if(depth.cols != img.cols || depth.rows != img.rows)
        return;
    cloud.width    = img.cols;
    cloud.height   = img.rows;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    depth.convertTo(depth,CV_32F,mDepthFactor);
    if(img.channels()==3)
    {
        cv::MatIterator_<cv::Vec3b> it_rgb;
        cv::MatIterator_<float> it_depth;
        int i;
        for (it_rgb = img.begin<cv::Vec3b>(),it_depth = depth.begin<float>(), i=0; it_rgb != img.end<cv::Vec3b>() ; it_rgb++,it_depth++,i++)
        {
          cloud.points[i].x = i/cloud.width;
          cloud.points[i].y = i%cloud.width;
          cloud.points[i].z = (*it_depth);
          cloud.points[i].r = (*it_rgb)[2];
          cloud.points[i].g = (*it_rgb)[1];
          cloud.points[i].b = (*it_rgb)[0];
        }
    *in_cloud = cloud;
    }

}
bool Integrater::saveTSDF(const char* path)
{
    mTSDF->save(path);
    return true;
}

bool Integrater::generateMesh(bool visualize)
{
    cpu_tsdf::MarchingCubesTSDFOctree mc;
    mc.setInputTSDF (mTSDF);
    mc.setMinWeight (1); // Sets the minimum weight -- i.e. if a voxel sees a point less than 2 times, it will not render  a mesh triangle at that location
    mc.setColorByRGB (true); // If true, tries to use the RGB values of the TSDF for meshing -- required if you want a colored mesh
    pcl::PolygonMesh mesh;
    mc.reconstruct (mesh);

    pcl::visualization::PCLVisualizer::Ptr vis;
    vis.reset (new pcl::visualization::PCLVisualizer);
    vis->addCoordinateSystem ();
    vis->removeAllPointClouds ();
    vis->addPolygonMesh (mesh);
    vis->spin ();
}
