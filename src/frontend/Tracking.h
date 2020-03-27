//
// Created by jojo on 25.03.20.
//

#ifndef BAMAPPING_TRACKING_H
#define BAMAPPING_TRACKING_H

#include "../Frame.h"
#include "../math/Types.h"

#include "DBoW2.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
namespace BAMapping
{
    namespace FrontEnd
    {
        class Tracking
        {
        public:
            static bool track(Frame& frame,const Frame* pRef_frame,std::vector<cv::DMatch> &inlier_matches);
            static bool match(const Frame& frame, const Frame& ref_frame, std::vector<cv::DMatch> &inlier_matches);
        private:
            static void ExtractORB(Frame& frame);
            static void MatchORB(const cv::Mat &query, const cv::Mat &target, std::vector<cv::DMatch> &goodMatches);
            static std::vector<cv::DMatch> GetSparsePointClouds(
                    const Frame& frame, const Frame& ref_frame,
                    const std::vector<cv::DMatch> &matches,
                    std::vector<Vec3>& frame_pt,std::vector<Vec3>& ref_frame_pt);
        };
    }
}



#endif //BAMAPPING_TRACKING_H
