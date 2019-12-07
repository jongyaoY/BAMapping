//
// Created by jojo on 06.12.19.
//

#include "Point.h"

using namespace BAMapping;

Point::Point(Eigen::Vector3d pose)
{
    mPose = pose;
}
void Point::setPoint(Eigen::Vector3d pose)
{
    mPose = pose;
}


void Point::getMutable(double* param)
{
    *(param + 0) = mPose[0];
    *(param + 1) = mPose[1];
    *(param + 2) = mPose[2];
}

const Eigen::Vector3d Point::getPoseInWorld()
{
    return mPose;
}

const Eigen::Vector3d Point::getPoseInFrame(const Eigen::Matrix4d Tcw)
{
    Eigen::Vector4d pose;
//    Eigen::Vector3d p;
//    Eigen::Affine3d T;
//    T.matrix() = Twc;
//    p = mPose;
//    p = T*p;
    pose[0] = mPose[0];
    pose[1] = mPose[1];
    pose[2] = mPose[2];
    pose[3] = 1;
    pose = Tcw*pose;
    return Eigen::Vector3d(pose[0],pose[1],pose[2]);
}