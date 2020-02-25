//
// Created by jojo on 22.02.20.
//

#ifndef BAMAPPING_FRONTEND_H
#define BAMAPPING_FRONTEND_H

#include "Frame.h"
//#include "Viewer.h"
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
        class MapPoint
        {
        public:
            MapPoint() {}
            MapPoint(Vec3 pose):pose_(pose){ }
            Vec3 pose_;
            cv::Mat descritor_;
        };
        class Map
        {
        public:
            Map()
            {
                matcher_ = new cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
//                matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
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
//                cv::Mat descriptors;
//                for(const auto& point : mapPoints)
//                {
//                    descriptors.push_back(point.descritor_);
//                }
//                matcher_->add(descriptors);
//                matcher_->train();
            }
            void addMapPoint(MapPoint point)
            {
                mapPoints_.push_back(point);
//                matcher_->add(point.descritor_);
//                matcher_->train();
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
        void ExtractAllFeaturesAndTrainMatcher(FrameVector& frameVector);
        static void ExtractAllFeaturesAndCreateDB(FrameVector& frameVector, std::string db_name);
        static void ExtractFeatures(FrameVector& frameVector);
        void ExtractORB(Frame& frame,cv::Ptr<cv::ORB> orb_detector);
        void CreatePointCloud(FrameVector& frameVector);
        void ExtractAndMatchFeatures(FrameVector& frameVector);
        void query(const Frame& frame,std::vector<cv::DMatch> &goodMatches);
        Map mMap;
    private:
        bool createNewPoint(MapPoint& mapPoint, const cv::KeyPoint& point, const cv::Mat& descriptor,const Frame& frame, const cv::Mat& depth_img, const double depth_thres = 3.0);

        void initMap(Frame& first_frame);

        void updateMap(Frame& frame, const Frame& ref_frame, std::vector<cv::DMatch> matches);

        void matchORB(const cv::Mat &query, const cv::Mat &target,
                      std::vector<cv::DMatch> &goodMatches);
        void matchFeatures(const cv::Mat &query, const cv::Mat &target,
                                std::vector<cv::DMatch> &goodMatches);

    };
}




#endif //BAMAPPING_FRONTEND_H
