//
// Created by jojo on 06.12.19.
//

#include <iostream>
#include "Frame.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
using namespace BAMapping;


std::map<size_t, Eigen::Vector3d> Frame::getObservations()
{
    return mObservations;
}

void Frame::setAngleAxisAndPoint(Eigen::AngleAxisd angleAxis, Eigen::Vector3d point)
{
    m_angleAxis = angleAxis;
    m_translation = point;
}
std::list<size_t> Frame::getObservedPointsIds()
{
    std::list<size_t > ids;
    for(auto obs : mObservations)
    {
        ids.push_back(obs.first);
    }
    return ids;
}

void Frame::setFromQuaternionAndPoint(Eigen::Quaterniond q, Eigen::Vector3d point)
{
    m_angleAxis = Eigen::AngleAxisd(q);
    m_translation = point;
}

void Frame::setTimeStamp(const double timeStampe)
{
    mTimeStamp = timeStampe;
}

void Frame::setIntrinsics(const double fx, const double fy, const double cx, const double cy)
{
    m_fx = fx;
    m_fy = fy;
    m_cx = cx;
    m_cy = cy;
}

void Frame::setDistortionFactors(const double k1, const double k2)
{
    m_k1 = k1;
    m_k2 = k2;
}

void Frame::addObservation(size_t point_id, Eigen::Vector3d observation)
{
    mObservations.emplace(point_id,observation);
}

void Frame::setImagePaths(const char *rgb_path, const char *depth_path, const char* infraRead_path)
{
    m_rgbImgPath = rgb_path;
    m_depthImgPath = depth_path;
    if(infraRead_path == NULL)
        m_infraRedImgPath = "";
    else
        m_infraRedImgPath = infraRead_path;
}

const Eigen::Matrix4d Frame::getConstTwc() const
{
    return Converter::AngleAxisPointToTwc(m_angleAxis,m_translation);
}

const Eigen::Matrix4d Frame::getConstTcw() const
{
    return Converter::AngleAxisPointToTcw(m_angleAxis,m_translation);
}

bool Frame::isPointObserved(size_t global_point_id)
{
    auto it = mObservations.find(global_point_id);
    if(it!=mObservations.end())
        return true;
    else
        return false;
}

Eigen::Vector3d Frame::getObservationByPointIndex(size_t global_point_id)
{
    auto it = mObservations.find(global_point_id);
    if(it!=mObservations.end())
        return it->second;
}

void Frame::setFromAffine3d(Eigen::Affine3d Tcw)
{
    Converter::Affine3dTcwToAngleAxisAndPoint(Tcw,m_angleAxis,m_translation);
}


void Frame::generateObservations()
{
    using namespace cv;
    Mat depth_img = imread(m_depthImgPath,IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img,CV_32F);
    depth_img /= 1000.0; //todo
    mObservations.clear();
    for(int i = 0; i < mKeypoints.size(); i++)
    {
        const auto& point = mKeypoints[i];
        const auto id = mKeyPointGlobalIds[i];
        if(id < 0) // invalid point
        {
            continue;
        }
        double u = point.pt.x;
        double v = point.pt.y;
        double d = depth_img.at<float>(point.pt);
        mObservations.emplace(id,Eigen::Vector3d(u,v,d));
    }
}

//intrisic parameters
double Frame::m_fx = 431.828094;
double Frame::m_fy = 431.828094;
double Frame::m_cx = 323.000610;
double Frame::m_cy = 240.218506;
//distortion coefficients
double Frame::m_k1;
double Frame::m_k2;