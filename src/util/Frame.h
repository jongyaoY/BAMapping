#ifndef FRAME_H
#define FRAME_H
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "Point.h"

class Frame
{
public:
    typedef Eigen::Matrix<float,4,4,Eigen::ColMajor> Pose;
    typedef std::pair<unsigned int,Eigen::Vector3d> Observation;
    typedef std::vector<Observation> ObservationVector;
    typedef enum
    {
        Quaternion,
        AxisAngle,
        EulerAngle,
        RotationMatrix
    } RotationType;

    Frame();
//    Frame(Frame frame);
    Frame(Pose Tcw):m_Tcw(Tcw){}
    void addObservation(Observation obs);
    double* getMutable(RotationType rotType = RotationType::Quaternion,bool withIntrinsic = false);
    const ObservationVector getObservations()const {return m_Observations;}

    inline void setPose(Pose Tcw){m_Tcw = Tcw;}
    inline const Pose getConstPose(){return m_Tcw;}


    float frameSize; //in meter
private:
    double m_timeStamp;
    Pose m_Tcw;
    double *m_mutableParam;
    //intrisic parameters
    double fx;
    double fy;
    double cx;  //principle point x
    double cy;  //principle point y
    //distortion coefficients
    double k1;
    double k2;
    double k3;
    ObservationVector m_Observations;

    std::string m_rgbImgPath;
    std::string m_depthImgPath;
};

typedef std::vector<Frame> FrameVector;
typedef std::vector<Frame*> FramePtrVector;
#endif // FRAME_H
