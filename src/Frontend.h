//
// Created by jojo on 22.02.20.
//

#ifndef BAMAPPING_FRONTEND_H
#define BAMAPPING_FRONTEND_H

#include "Frame.h"
//#include "Viewer.h"
#include "MapPoint.h"
#include "DBoW2.h"
#include "math/Types.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "DBoW2.h"

namespace BAMapping
{
    class Frontend
    {
    public:
        class Map
        {
        public:
            Map()
            {
                matcher_ = new cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
            }
            void init(const cv::Mat& descriptors,const std::vector<MapPoint>& mapPoints)
            {
                mapPoints_ = mapPoints;
                matcher_->add(descriptors);
                matcher_->train();
            }
            void addPoints(const std::vector<MapPoint>& mapPoints)
            {
                mapPoints_.insert(mapPoints_.end(),mapPoints.begin(),mapPoints.end());

            }
            void addMapPoint(MapPoint point)
            {
                mapPoints_.push_back(point);

            }
            void matchFeatures(const cv::Mat &query, std::vector<cv::DMatch> &goodMatches)
            {
                using namespace cv;
                std::vector<std::vector<cv::DMatch>> matches;

//                matcher_->match(query,goodMatches);
                matcher_->knnMatch(query,matcher_->getTrainDescriptors(),matches,2);
//                 Second neighbor ratio test.

                for (unsigned int i = 0; i < matches.size(); ++i)
                {
                    if(matches[i].empty())
                        continue;

                    if (matches[i][0].distance < matches[i][1].distance * 0.75)
                        goodMatches.push_back(matches[i][0]);
                }
            }
            std::vector<MapPoint> mapPoints_;
            cv::Ptr<cv::FlannBasedMatcher> matcher_;
        };

        class MotionModel
        {
        public:
            MotionModel() {}
//            MotionModel(const FrameVector& frameVector)
//            {
//                for(const auto& frame : frameVector)
//                {
//                    node_Twc.push_back(frame.getConstTwc());
//                }
//            }
            void init(const FrameVector& frameVector)
            {
                for(const auto& frame : frameVector)
                {
                    node_Twc.push_back(frame.getConstTwc());
                }
            }
            Mat4 getTij(size_t i, size_t j)
            {
                if(i > node_Twc.size() || j > node_Twc.size())
                {
                    std::cout<<"node id exceeds"<<std::endl;
                    return Mat4::Identity();
                }
                auto Tiw = node_Twc[i].inverse();
                auto Twj = node_Twc[j];
                return Tiw * Twj;
            }

            std::vector<Mat4> node_Twc;
        private:
        };

        void ExtractORB(Frame& frame,cv::Ptr<cv::ORB> orb_detector);
        void ExtractAndMatchFeatures(FrameVector& frameVector, const std::string voc_path);
        void ExtractAndCreateDatabase(FrameVector& frameVector, const std::string voc_path, const std::string db_path = "");
        void LoadDataBase(const std::string db_path);
        void query(const Frame& frame, DBoW2::QueryResults& ret);
        void matchORB(const cv::Mat &query, const cv::Mat &target,
                      std::vector<cv::DMatch> &goodMatches);
        Map mMap;
    private:
        bool createNewPoint(MapPoint& mapPoint, const cv::KeyPoint& point, const cv::Mat& descriptor,const Frame& frame, const cv::Mat& depth_img, const double depth_thres = 3.0);

        void updateMap(Frame& frame,
                Frame& ref_frame, const std::vector<cv::DMatch>& matches);

        int detectLoopClosure(OrbDatabase& db,const Frame& frame);


        void alignFrames(Frame& frame, const Mat4 last_Twc ,const Frame& ref_frame,const std::vector<cv::DMatch>& matches);

        void removeOutliers(const Frame& frame_query, const Frame& frame_train, std::vector<cv::DMatch>& matches, const double thres = 0.5);

        void triangulateKeyPoints(const Frame& frame, const Frame& ref_frame, const std::vector<cv::DMatch>& matches, std::vector<MapPoint>& mapPoints);

        MotionModel motionModel_;
        OrbDatabase feature_db_;
    };
}




#endif //BAMAPPING_FRONTEND_H
