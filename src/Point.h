//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_POINT_H
#define BAMAPPING_POINT_H

#include <map>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace BAMapping
{
    class Point
    {
    public:
        Point() = default;
        Point(Eigen::Vector3d pose);
        ~Point() = default;

        void setPoint(Eigen::Vector3d pose);

        void getMutable(double* param);
        const Eigen::Vector3d getPoseInWorld();
        const Eigen::Vector3d getPoseInFrame(const Eigen::Affine3d Twc);
    private:
        Eigen::Vector3d mPose;
        size_t mIndex;
    };
    typedef std::vector<Point> PointVector;
    typedef std::vector<Point*> PointPtrVector;
} //end of namespace




#endif //BAMAPPING_POINT_H
