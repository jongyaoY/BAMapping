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

#include "MapPoint.h"
#include "Point.h"
#include "util/Converter.h"
#include "util/Parser.h"
#include "opencv2/core.hpp"
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
        std::map<size_t ,Eigen::Vector3d> getObservations();
        const Eigen::Matrix4d getConstTwc() const;
        const Eigen::Matrix4d getConstTcw() const;
        const Eigen::AngleAxisd getConstAngleAxis();
        const Eigen::Vector3d getConstTranslation() const {return m_translation;};
        inline double getTimeStamp() const{return mTimeStamp;}

        void setImagePaths(const char* rgb_path,const char* depth_path,const char* infraRead_path = NULL);
        inline std::string getRGBImagePath()const {return m_rgbImgPath;}
        inline std::string getDepthImagePath()const {return m_depthImgPath;}
        inline std::string getInfraRedImagePath()const {return m_infraRedImgPath;}

        void generateObservations();

        size_t mGlobalIndex;
        std::vector<size_t> mKeyPointGlobalIds;
        std::vector<bool> keyPoint_has_match;
        std::vector<cv::KeyPoint> mKeypoints;
        std::vector<MapPoint::Ptr> mpMapPoints;
        cv::Mat mDescriptior;
        std::vector<cv::Mat> mKepoint_descriptors;
        std::map<size_t ,Eigen::Vector3d> mObservations;
        size_t mITEId;

        //intrisic parameters
        static double m_fx;
        static double m_fy;
        static double m_cx;  //principle point x
        static double m_cy;  //principle point y

    private:
        double mTimeStamp;
        std::string m_rgbImgPath;
        std::string m_depthImgPath;
        std::string m_infraRedImgPath;
        //for optimization
        Eigen::AngleAxisd m_angleAxis;
        Eigen::Vector3d m_translation;


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

        static void filterObservations(FrameVector& frameVector, PointVector& pointVector, const char* config_file)
        {
            Parser config(config_file);

            double diff_thres = config.getValue<double>("correspondence_diff_thres");
            double fx = config.getValue<double>("Camera.fx");
            double fy = config.getValue<double>("Camera.fy");
            double cx = config.getValue<double>("Camera.cx");
            double cy = config.getValue<double>("Camera.cy");

            for(auto& frame : frameVector)
            {
                Eigen::Affine3d Twc;
                Twc.matrix() = frame.getConstTwc();
                std::map<size_t ,Eigen::Vector3d> new_obs_vec;
                std::map<size_t ,Eigen::Vector3d>::iterator it = frame.mObservations.begin();
                for(; it != frame.mObservations.end(); it++)
                {
                    auto point_id = it->first;
                    auto obs = it->second;
                    Eigen::Vector3d p_obs;
                    Eigen::Vector3d p_world;
                    Eigen::Vector3d diff;

                    p_obs[0] = (obs[0]-cx)*obs[2]/fx;
                    p_obs[1] = (obs[1]-cy)*obs[2]/fy;
                    p_obs[2] = obs[2];

//                    p_obs = Twc * p_obs;
                    p_world = pointVector[point_id].getPoseInWorld();
                    p_world = Twc.inverse() * p_world;
                    diff = p_obs - p_world;
                    if(sqrt(diff.dot(diff)) > diff_thres)
                    {
//                        frame.mObservations.erase(it);
                        printf("removed obs, diff: %lf\n", sqrt(diff.dot(diff)));
                    }
                    else
                    {
                        new_obs_vec.emplace(*it);
                    }
                }
                frame.mObservations.clear();
                frame.mObservations = new_obs_vec;
            }

            int i = 0;
            FrameVector out_frameVector;

            for(const auto& frame : frameVector)
            {
                if(frame.mObservations.size() < 15)
                {
                    printf("removed %d, size : %lu\n",i, frame.mObservations.size());
                }
                else
                {
                    out_frameVector.push_back(frame);
                }
                i++;
            }
//            for(FrameVector::iterator it = frameVector.begin(); it != frameVector.end(); it++)
//            {
//                if(it->mObservations.size() < 15)
//                {
//                    printf("removed %d\n",i);
//                }
//                else
//                {
//                    out_frameVector.push_back(*it);
//                }
//                i++;
//            }
            frameVector.clear();
            frameVector = out_frameVector;

        }

        static void findCorrespondence();
    }

}//end of namespace




#endif //BAMAPPING_FRAME_H
