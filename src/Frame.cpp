//
// Created by jojo on 06.12.19.
//

#include "Frame.h"

using namespace BAMapping;

void Frame::getMutable(double* param)
{
    for(int i = 0; i < 3; i++)
    {
        *(param + i) = m_angleAxis.angle()*m_angleAxis.axis()[i];
    }
    for(int i = 0; i < 3; i++)
    {
        *(param + i + 3) = m_translation[i];
    }
}

std::map<size_t, Eigen::Vector3d> Frame::getObservations()
{
    return mObservations;
}

void Frame::setAngleAxisAndPoint(Eigen::AngleAxisd angleAxis, Eigen::Vector3d point)
{
    m_angleAxis = angleAxis;
    m_translation = point;
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

void Frame::setImagePaths(const char *rgb_path, const char *depth_path)
{
    m_rgbImgPath = rgb_path;
    m_depthImgPath = depth_path;
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

//intrisic parameters
double Frame::m_fx;
double Frame::m_fy;
double Frame::m_cx;
double Frame::m_cy;
//distortion coefficients
double Frame::m_k1;
double Frame::m_k2;