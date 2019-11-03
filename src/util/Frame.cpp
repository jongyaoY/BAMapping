#include "Frame.h"

Frame::Frame()
{
    m_Tcw = Pose::Identity();
    frameSize = 0.01;
    m_timeStamp = 0;
}
void Frame::addObservation(Observation obs)
{
    m_Observations.push_back(obs);
}
double* Frame::getMutable(RotationType rotType,bool withIntrinsic)
{
    m_mutableParam = new double[10];
    if(rotType == RotationType::Quaternion)
    {
        Eigen::Matrix3d rot;
        rot << m_Tcw(0,0),m_Tcw(0,1),m_Tcw(0,2),
               m_Tcw(1,0),m_Tcw(1,1),m_Tcw(1,2),
               m_Tcw(2,0),m_Tcw(2,1),m_Tcw(2,2);
        Eigen::Quaterniond q(rot);
        m_mutableParam[0] = q.x();
        m_mutableParam[1] = q.y();
        m_mutableParam[2] = q.z();
        m_mutableParam[3] = q.w();

        m_mutableParam[4] = m_Tcw(3,0);
        m_mutableParam[5] = m_Tcw(3,1);
        m_mutableParam[6] = m_Tcw(3,2);

        m_mutableParam[7] = 0; //focal length
        m_mutableParam[8] = 0; //first order distortion
        m_mutableParam[9] = 0; //second order distortion
    }
    return m_mutableParam;
}

