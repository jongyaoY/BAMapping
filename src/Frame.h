//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_FRAME_H
#define BAMAPPING_FRAME_H

#include <vector>
#include <map>
#include <memory>
#include <Eigen/Geometry>
#include <list>
#include <set>


#include "util/Converter.h"
#include "util/Parser.h"
namespace BAMapping
{
    class Frame
    {
    public:
        typedef std::shared_ptr<Frame> Ptr;

        Frame() = default;
        void setAngleAxisAndPoint(Eigen::AngleAxisd angleAxis,Eigen::Vector3d point);
        void setFromQuaternionAndPoint(Eigen::Quaterniond q, Eigen::Vector3d point);
        void setFromAffine3d(const Eigen::Affine3d Tcw);

        void setTimeStamp(const double timeStampe);

        static void setIntrinsics(const double fx, const double fy, const double cx, const double cy);
        static void setDistortionFactors(const double k1, const double k2);

        void addObservation(size_t point_id,Eigen::Vector3d observation);

        bool isPointObserved(size_t global_point_id);
        std::list<size_t> getObservedPointsIds();
        Eigen::Vector3d getObservationByPointIndex(size_t global_point_id);
        void getIntrinsics(double& fx,double& fy,double& cx,double& cy);
        std::map<size_t ,Eigen::Vector3d> getObservations();
        void getMutable(double* param);
        const Eigen::Matrix4d getConstTwc() const;
        const Eigen::Matrix4d getConstTcw() const;
        const Eigen::AngleAxisd getConstAngleAxis();
        const Eigen::Vector3d getConstTranslation();
        inline double getTimeStamp() const{return mTimeStamp;}

        void setImagePaths(const char* rgb_path,const char* depth_path,const char* infraRead_path = NULL);
        inline std::string getRGBImagePath()const {return m_rgbImgPath;}
        inline std::string getDepthImagePath()const {return m_depthImgPath;}
        inline std::string getInfraRedImagePath()const {return m_infraRedImgPath;}

        size_t mGlobalIndex;

    private:
        double mTimeStamp;
        std::string m_rgbImgPath;
        std::string m_depthImgPath;
        std::string m_infraRedImgPath;
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

    namespace FrameMethods
    {
        static FrameVector filterFrames(const char* config_file, const FrameVector& in_frameVector)
        {

            Parser config(config_file);
            FrameVector out_frameVector;
            if(in_frameVector.empty())
                return out_frameVector;

            auto n_key_points_thres = config.getValue<int>("Frame.n_key_points_thres");
            auto tracked_key_points_percentage_thres = config.getValue<double>("Frame.tracked_key_points_percentage_thres");

            Frame ref_frame;
            for(auto frame : in_frameVector)
            {
                if(!frame.getObservedPointsIds().empty())
                {
                    ref_frame = frame;
                    break;
                }
            }
            for(auto frame : in_frameVector)
            {
                auto ref_tracked_points = ref_frame.getObservedPointsIds();
                auto tracked_points = frame.getObservedPointsIds();
                if(tracked_points.empty())
                    continue;

                ref_tracked_points.sort();
                tracked_points.sort();
                std::set<int> intersect;
                std::set_intersection(ref_tracked_points.begin(),ref_tracked_points.end(),tracked_points.begin(),tracked_points.end(),
                                      std::inserter(intersect,intersect.begin()));
                double percentage = (double) intersect.size()/(double) ref_tracked_points.size();
                if(percentage <  tracked_key_points_percentage_thres)
                {
                    ref_frame = frame;
                    out_frameVector.push_back(frame);
                }
            }

            return out_frameVector;
        }
    }

}//end of namespace




#endif //BAMAPPING_FRAME_H
