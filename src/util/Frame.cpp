#include "Frame.h"

Frame::Frame()
{
    m_k1 = 0.;
    m_k2 = 0.;
    m_timeStamp = 0;
}
void Frame::setPose(Pose pose)
{

}

void Frame::setAngleAxisAndPoint(Eigen::AngleAxisd angleAxis,Eigen::Vector3d point)
{
    m_angleAxis = angleAxis;
    m_translation = point;
}

void Frame::setFromQuaternionAndPoint(Eigen::Quaterniond q, Eigen::Vector3d t)
{
    m_angleAxis = Eigen::AngleAxisd(q);
    m_translation = t;
}
void Frame::addObservation(Observation obs)
{
    m_Observations.push_back(obs);
}
void Frame::addObservation(unsigned int point_id,double u,double v,double d)
{
    Observation obs;
    obs.first = point_id;
    obs.second = Eigen::Vector3d(u,v,d);
//    obs.second[1] = v;
//    obs.second[2] = d;
    m_Observations.push_back(obs);
}
template<>
void Frame::setTimeStamp(const double timeStampe)
{
    m_timeStamp = timeStampe;
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

void Frame::getIntrinsics(double& fx,double& fy,double& cx,double& cy)
{
    fx = m_fx;
    fy = m_fy;
    cx = m_cx;
    cy = m_cy;
}

void Frame::getMutable(double* param, bool withIntrinsics)
{
    for(int i = 0; i < mRotationBlockSize; i++)
    {
        *(param + i) = m_angleAxis.angle()*m_angleAxis.axis()[i];
    }
    for(int i = 0; i < 3; i++)
    {
        *(param + i + mRotationBlockSize) = m_translation[i];
    }
    if(!withIntrinsics)
        return;

    *(param + 6) = m_f; //focal length
    *(param + 7) = m_k1; //first order distortion
    *(param + 8) = m_k2; //second order distortion
}

const Frame::Pose Frame::getConstTwc() const
{
    Pose Twc = Pose::Identity();
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

const Frame::Pose Frame::getConstTcw() const
{
    Pose Tcw = Pose::Identity();
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

const Eigen::AngleAxisd Frame::getConstAngleAxis()
{
    return m_angleAxis;
}

const Eigen::Vector3d Frame::getConstTranslation()
{
    return m_translation;
}

int Frame::getParamBlockSize()
{
    return mParamBlockSize;
}

void Frame::setImagePaths(const char* rgb_path,const char* depth_path)
{
    m_rgbImgPath = rgb_path;
    m_depthImgPath = depth_path;
}

int Frame::mParamBlockSize = 9;
int Frame::mRotationBlockSize = 3;
int Frame::mIntrinsicsBlockSize = 3;


