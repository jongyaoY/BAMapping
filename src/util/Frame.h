#ifndef FRAME_H
#define FRAME_H
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "Point.h"

class Frame
{
public:
    Frame();
//    typedef Eigen::Quaterniond Quaternion;
    typedef Eigen::Matrix<float,4,4,Eigen::ColMajor> Pose;
    typedef std::pair<unsigned int,Eigen::Vector2f> Observation;
    typedef std::vector<Observation> ObservationVector;
    typedef enum
    {
        Quaternion,
        AxisAngle
    } RotationType;

    double* getMutable(RotationType rotType = RotationType::Quaternion,bool withIntrinsic = false);
    const ObservationVector getObservations()const {return z_vec;}
    void updateFrame();


    double timeStamp;
    Pose Tcw;
    ObservationVector z_vec;
    PointVector featurePoint_vec;
    //MapPointVector mapPoint_vec; //to do
    float frameSize; //in meter
private:
    double *m_mutableParam;
    double fx;
    double fy;
    double cx;
    double cy;

};

typedef std::vector<Frame> FrameVector;
#endif // FRAME_H
