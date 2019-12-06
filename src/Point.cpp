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
