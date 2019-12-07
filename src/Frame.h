//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_FRAME_H
#define BAMAPPING_FRAME_H

#include <vector>
#include <map>
#include <memory>
#include <Eigen/Geometry>

#include "util/Converter.h"
namespace BAMapping
{
    class Frame
    {
    public:
        typedef std::shared_ptr<Frame> Ptr;

        Frame() = default;
//        Frame(Frame const &frame);
        void setAngleAxisAndPoint(Eigen::AngleAxisd angleAxis,Eigen::Vector3d point);
        void setFromQuaternionAndPoint(Eigen::Quaterniond q, Eigen::Vector3d point);

        void setTimeStamp(const double timeStampe);

        static void setIntrinsics(const double fx, const double fy, const double cx, const double cy);
        static void setDistortionFactors(const double k1, const double k2);

        void addObservation(size_t point_id,Eigen::Vector3d observation);

        void getIntrinsics(double& fx,double& fy,double& cx,double& cy);
        std::map<size_t ,Eigen::Vector3d> getObservations();
        void getMutable(double* param);
        const Eigen::Matrix4d getConstTwc() const;
        const Eigen::Matrix4d getConstTcw() const;
        const Eigen::AngleAxisd getConstAngleAxis();
        const Eigen::Vector3d getConstTranslation();
        inline double getTimeStamp() const{return mTimeStamp;}

        void setImagePaths(const char* rgb_path,const char* depth_path);
        inline std::string getRGBImagePath()const {return m_rgbImgPath;}
        inline std::string getDepthImagePath()const {return m_depthImgPath;}


    private:
        double mTimeStamp;
        std::string m_rgbImgPath;
        std::string m_depthImgPath;
        //for optimization
        Eigen::AngleAxisd m_angleAxis;
        Eigen::Vector3d m_translation;
        std::map<size_t ,Eigen::Vector3d> mObservations;

        //intrisic parameters
        static double m_fx;
        static double m_fy;
        static double m_cx;  //principle point x
        static double m_cy;  //principle point y
        //distortion coefficients
        static double m_k1;   //(u',v') = (cx,cy) + (1 + k1*r^2 + k2*r^4)*(u-cx,v-cy);
        static double m_k2;   //r^2 = (u - cx)^2 + (v - cy)^2


    };
    typedef std::vector<Frame> FrameVector;
    typedef std::vector<Frame*> FramePtrVector;
//    typedef std::vector<Frame*> FramePtrVector;

}//end of namespace




#endif //BAMAPPING_FRAME_H
