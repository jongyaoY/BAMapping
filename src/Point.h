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
        void addObservation(size_t global_frame_index,Eigen::Vector3d obs);
//        bool isObservedByFrame(size_t global_frame_id);
        size_t getObservationSize();
        std::map<size_t,Eigen::Vector3d> getObservations();

        void getMutable(double* param);
        const Eigen::Vector3d getPoseInWorld();
        const Eigen::Vector3d getPoseInFrame(const Eigen::Matrix4d Tcw);

        size_t mGlobalIndex;
    private:
        Eigen::Vector3d mPose;
        std::map<size_t,Eigen::Vector3d> mObservations;
    };
    typedef std::vector<Point> PointVector;
    typedef std::vector<Point*> PointPtrVector;
} //end of namespace




#endif //BAMAPPING_POINT_H
