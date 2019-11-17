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

    Frame();

    void addObservation(Observation obs);
    inline void setPose(Pose Tcw);
    void setAngleAxisAndPoint(Eigen::AngleAxisd angleAxis,Eigen::Vector3d point);
    void setFromQuaternionAndPoint(Eigen::Quaterniond q, Eigen::Vector3d t);
    template<typename T>
    void setTimeStamp(const T timeStampe);
    template<typename T>
    void setIntrinsics(const T fx, const T fy, const T cx, const T cy);
    template<typename T>
    void setDistortionFactors(const T k1, const T k2);

    void getMutable(double* param);
    const ObservationVector getObservations()const {return m_Observations;}
    const unsigned int getObservationSize()const {return m_Observations.size();}
    static int getParamBlockSize();
    const Pose getConstTwc() const;
    const Pose getConstTcw() const;
    const Eigen::AngleAxisd getConstAngleAxis();
    const Eigen::Vector3d getConstTranslation();
    inline double getTimeStamp() const{return m_timeStamp;}

    void setImagePaths(const char* rgb_path,const char* depth_path);
    inline std::string getRGBImagePath()const {return m_rgbImgPath;}
    inline std::string getDepthImagePath()const {return m_depthImgPath;}
private:
    double m_timeStamp;
    //for optimization
    Eigen::AngleAxisd m_angleAxis;
    Eigen::Vector3d m_translation;
    static int mParamBlockSize;
    static int mRotationBlockSize;
    static int mIntrinsicsBlockSize;
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
