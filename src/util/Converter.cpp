//
// Created by jojo on 07.12.19.
//

#include "Converter.h"

using namespace BAMapping;


Eigen::Matrix4d Converter::AngleAxisPointToTcw(const Eigen::AngleAxisd angleAxis, const Eigen::Vector3d point)
{
    Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    R = angleAxis.toRotationMatrix();
    t = point;
    setRotToMatrix4d(R,Tcw);
    setTransToMatrix4d(t,Tcw);
    return Tcw;
}


Eigen::Matrix4d Converter::AngleAxisPointToTwc(const Eigen::AngleAxisd angleAxis, const Eigen::Vector3d point)
{
    Eigen::Matrix4d Twc =  Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    auto angleAxis_inv = angleAxis;

    R = angleAxis.toRotationMatrix().transpose();
    t = R*point;
    t*=-1;
    setRotToMatrix4d(R,Twc);
    setTransToMatrix4d(t,Twc);
    return Twc;

}

void Converter::copyMatrix4d(const Eigen::Matrix4d from, Eigen::Matrix4d& to)
{
    to.topLeftCorner(3,3)<<from(0,0),from(0,1),from(0,2),from(0,3),
                                        from(1,0),from(1,1),from(1,2),from(1,3),
                                        from(2,0),from(2,1),from(2,2),from(2,3);
                                        from(3,0),from(3,1),from(3,2),from(3,3);

}

Eigen::Matrix4d Converter::inverseT(Eigen::Matrix4d T)
{
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    R = T.topLeftCorner(3,3);
    R = R.transpose();
    t = T.topRightCorner(3,1);
    t = -R*t;
    setRotToMatrix4d(R,T_inv);
    setTransToMatrix4d(t,T_inv);
//    T_inv.topLeftCorner(3,3)<<R(0,0),R(0,1),R(0,2),
//            R(1,0),R(1,1),R(1,2),
//            R(2,0),R(2,1),R(2,2);
//    T_inv.topRightCorner(3,1)<<t[0],t[1],t[2];

    return T_inv;
}

void Converter::setRotToMatrix4d(const Eigen::Matrix3d R,Eigen::Matrix4d& T)
{
    T.topLeftCorner(3,3)<<R(0,0),R(0,1),R(0,2),
                                       R(1,0),R(1,1),R(1,2),
                                       R(2,0),R(2,1),R(2,2);
}

void Converter::setTransToMatrix4d(Eigen::Vector3d t,Eigen::Matrix4d& T)
{
    T.topRightCorner(3,1)<<t[0],t[1],t[2];
}

Eigen::Matrix3d Converter::getRotFromMatrix4d(const Eigen::Matrix4d T)
{
    return Eigen::Matrix3d();//todo
}

Eigen::Vector3d Converter::getTransFromMatrix4d(const Eigen::Matrix4d T)
{
    return Eigen::Vector3d();//todo
}

void Converter::Affine3dTcwToAngleAxisAndPoint(const Eigen::Affine3d Tcw, Eigen::AngleAxisd &angleAxis,
                                               Eigen::Vector3d &point)
{
    angleAxis.fromRotationMatrix(Tcw.rotation());
    point = Tcw.translation();
}
