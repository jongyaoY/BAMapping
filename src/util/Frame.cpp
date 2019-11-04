#include "Frame.h"

Frame::Frame()
{
    m_Tcw = Pose::Identity();
    m_k1 = 0.;
    m_k2 = 0.;
    frameSize = 0.01;
    m_timeStamp = 0;
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

double* Frame::getMutable(RotationType rotType)
{
    m_mutableParam = new double[10];
    if(rotType == RotationType::Quaternion)
    {
        Eigen::Matrix3d rot;
        rot << m_Tcw(0,0),m_Tcw(0,1),m_Tcw(0,2),
               m_Tcw(1,0),m_Tcw(1,1),m_Tcw(1,2),
               m_Tcw(2,0),m_Tcw(2,1),m_Tcw(2,2);
        Eigen::Quaterniond q(rot);
        m_mutableParam[0] = q.w();
        m_mutableParam[1] = q.x();
        m_mutableParam[2] = q.y();
        m_mutableParam[3] = q.z();

        m_mutableParam[4] = m_Tcw(3,0);
        m_mutableParam[5] = m_Tcw(3,1);
        m_mutableParam[6] = m_Tcw(3,2);

        m_mutableParam[7] = m_f; //focal length
        m_mutableParam[8] = m_k1; //first order distortion
        m_mutableParam[9] = m_k2; //second order distortion
    }
    return m_mutableParam;
}

