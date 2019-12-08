//
// Created by jojo on 07.12.19.
//

#ifndef BAMAPPING_CONVERTER_H
#define BAMAPPING_CONVERTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace BAMapping
{
    class Converter
    {
    public:
        static Eigen::Matrix4d AngleAxisPointToTwc(const Eigen::AngleAxisd angleAxis,const Eigen::Vector3d point);
        static Eigen::Matrix4d AngleAxisPointToTcw(const Eigen::AngleAxisd angleAxis,const Eigen::Vector3d point);
        static void Affine3dTcwToAngleAxisAndPoint(const Eigen::Affine3d Tcw,Eigen::AngleAxisd& angleAxis,Eigen::Vector3d& point);
//        static void Affine3dTwcToAngleAxisAndPoint(const Eigen::Affine3d Tcw,Eigen::AngleAxisd& angleAxis,Eigen::Vector3d& point);

        static Eigen::Matrix3d getRotFromMatrix4d(const Eigen::Matrix4d T);
        static Eigen::Vector3d getTransFromMatrix4d(const Eigen::Matrix4d T);
    private:
        static void copyMatrix4d(const Eigen::Matrix4d from,Eigen::Matrix4d& to);
        static void setRotToMatrix4d(const Eigen::Matrix3d R,Eigen::Matrix4d& T);
        static void setTransToMatrix4d(Eigen::Vector3d t,Eigen::Matrix4d& T);

        static Eigen::Matrix4d inverseT(Eigen::Matrix4d T);
    };
}



#endif //BAMAPPING_CONVERTER_H
