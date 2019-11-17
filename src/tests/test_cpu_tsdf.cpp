#include <stdio.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/io/ply_io.h"
#include <pcl/point_types.h>
#include "pcl/visualization/cloud_viewer.h"

#include "cpu_tsdf/tsdf_volume_octree.h"
#include "cpu_tsdf/marching_cubes_tsdf_octree.h"
#include "opencv4/opencv2/opencv.hpp"
//#include "opencv4/opencv2/imgcodecs.hpp"
#include "../Integrater.h"
#include "../Reader.h"
//#include "../Viewer.h"

void constructPose(Eigen::Affine3d& pose,const char* filename)
{
    FILE* pF = fopen(filename,"r");
    if(pF == NULL)
    {
        return;
    }
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    for(int i = 0;i < 3; i++)
    {
        double r[3],t_;
        fscanf(pF,"%lf %lf %lf %lf",&r[0],&r[1],&r[2],&t_);
        R.block<1,3>(i,0) << r[0],r[1],r[2];
        t[i] = t_;
    }
    pose.matrix().block<3,3>(0,0) = R;
    pose.translation() = t;

}
void createPCL(const char* img_name,pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
{
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    img = cv::imread(img_name,cv::IMREAD_COLOR);

    cloud.width    = img.cols;
    cloud.height   = img.rows;
    cloud.is_dense = true;
    cloud.points.resize (cloud.width * cloud.height);

    if(img.channels()==3)
    {
        cv::MatIterator_<cv::Vec3b> it;
        int i;
        for (it = img.begin<cv::Vec3b>(), i=0; it != img.end<cv::Vec3b>() ; it++,i++)
        {
          cloud.points[i].x = i/cloud.width;
          cloud.points[i].y = i%cloud.width;
          cloud.points[i].z = 10;
          cloud.points[i].r = (*it)[2];
          cloud.points[i].g = (*it)[1];
          cloud.points[i].b = (*it)[0];
//          cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
        }
//        pcl::visualization::PCLVisualizer::Ptr vis;
//        vis.reset (new pcl::visualization::PCLVisualizer);
//        vis->addCoordinateSystem ();
//        vis->removeAllPointClouds ();
//        vis->addPointCloud(cloud.makeShared());
//        vis->spin();
    *in_cloud = cloud;
    }

}

int main(int argc, char** argv)
{
    Integrater integrater;
    integrater.init(argv[1]);
    FrameVector frames;
    Reader::readTUMFrames(frames,"../dataset_local/fr1desk/","fr1_desk.txt","groundtruth.txt");
//    Viewer viewer;
//    viewer.setFrames(frames);
//    viewer.visualize();
    int num_max = 200;
    int num = 0;
    for(auto frame : frames)
    {
        std::cout<<"dealing frame:"<<num<<std::endl;
        integrater.integrateFrame(frame);
        if(num>num_max)
            break;
        num++;
    }
    integrater.saveTSDF("../dataset_local/tsdf.vol");
    integrater.generateMesh();
}
