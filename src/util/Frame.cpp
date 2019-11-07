#include "Frame.h"

Frame::Frame()
{
    m_k1 = 0.;
    m_k2 = 0.;
    frameSize = 0.05;
    m_timeStamp = 0;
}
void Frame::setPose(Pose pose)
{

}

void Frame::setAngleAxisAndPoint(Eigen::AngleAxisd angleAxis,Eigen::Vector3d point)
{
    m_angleAxis = angleAxis;m_translation = point;
}
void Frame::addObservation(Observation obs)
{
    m_Observations.push_back(obs);
}

template<>
void Frame::setIntrinsics(const double fx, const double fy, const double cx, const double cy)
{
    m_fx = (double) fx;
    m_fy = (double) fy;
    m_cx = (double) cx;
    m_cy = (double) cy;
}

template<>
void Frame::setDistortionFactors(const double k1, const double k2)
{
    m_k1 = (double) k1;
    m_k2 = (double) k2;
}

void Frame::getMutable(double* param)
{
    *(param + 0) = m_angleAxis.angle()*m_angleAxis.axis()[0];
    *(param + 1) = m_angleAxis.angle()*m_angleAxis.axis()[1];
    *(param + 2) = m_angleAxis.angle()*m_angleAxis.axis()[2];
    *(param + 3) = m_translation[0];
    *(param + 4) = m_translation[1];
    *(param + 5) = m_translation[2];
    *(param + 6) = m_f; //focal length
    *(param + 7) = m_k1; //first order distortion
    *(param + 8) = m_k2; //second order distortion
}


const Frame::Pose Frame::getConstTwc()
{
    Pose Twc;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    R = m_angleAxis.inverse().toRotationMatrix();
    t = R*m_translation;
    t*=-1;
    Twc.topLeftCorner(3,3)<<R(0,0),R(0,1),R(0,2),
                            R(1,0),R(1,1),R(1,2),
                            R(2,0),R(2,1),R(2,2);
    Twc.topRightCorner(3,1)<<t[0],t[1],t[2];
    Twc(3,3) = 1;
    return Twc;
}

const Frame::Pose Frame::getConstTcw()
{
    Pose Tcw;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    R = m_angleAxis.toRotationMatrix();
    t = m_translation;

    Tcw.topLeftCorner(3,3)<<R(0,0),R(0,1),R(0,2),
                            R(1,0),R(1,1),R(1,2),
                            R(2,0),R(2,1),R(2,2);
    Tcw.topRightCorner(3,1)<<t[0],t[1],t[2];
    Tcw(3,3) = 1;
    return Tcw;
}
