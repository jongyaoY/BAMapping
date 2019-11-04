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
    inline void setPose(Pose Tcw){m_Tcw = Tcw;}
    template<typename T>
    void setIntrinsics(const T fx, const T fy, const T cx, const T cy);
    template<typename T>
    void setDistortionFactors(const T k1, const T k2);

    double* getMutable(RotationType rotType = RotationType::Quaternion);
    const ObservationVector getObservations()const {return m_Observations;}
    const unsigned int getObservationSize()const {return m_Observations.size();}
    inline const Pose getConstPose(){return m_Tcw;}


    float frameSize; //in meter
private:
    double m_timeStamp;
    Pose m_Tcw;
    double *m_mutableParam;
    //intrisic parameters
    double m_f;   //focal length
    double m_fx;
    double m_fy;
    double m_cx;  //principle point x
    double m_cy;  //principle point y
    //distortion coefficients
    double m_k1;   //(u',v') = (cx,cy) + (1 + k1*r^2 + k2*r^4)*(u-cx,v-cy);
    double m_k2;   //r^2 = (u - cx)^2 + (v - cy)^2
    ObservationVector m_Observations;

    std::string m_rgbImgPath;
    std::string m_depthImgPath;
};

typedef std::vector<Frame> FrameVector;
typedef std::vector<Frame*> FramePtrVector;
#endif // FRAME_H
