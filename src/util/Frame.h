#ifndef FRAME_H
#define FRAME_H
#include <Eigen/Eigen>
#include "Point.h"

class Frame
{
public:
    Frame();
    typedef Eigen::Matrix<float,4,4,Eigen::ColMajor> Pose;
    typedef Eigen::Vector2f Observation;
    typedef std::vector<Observation> ObservationVector;

    double timeStamp;
    Pose Tcw;
    ObservationVector z_vec;
    PointVector featurePoint_vec;
    //MapPointVector mapPoint_vec; //to do
    float frameSize; //in meter

};

typedef std::vector<Frame> FrameVector;
#endif // FRAME_H
