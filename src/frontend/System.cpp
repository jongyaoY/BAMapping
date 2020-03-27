//
// Created by jojo on 25.03.20.
//

#include "System.h"
#include "LoopClosing.h"

#include <opencv2/core/persistence.hpp>
#include <random>

using namespace BAMapping::FrontEnd;

void System::run(FrameVector &frameVector, const std::string config_file)
{
    using namespace cv;
    FileStorage fs(config_file,FileStorage::READ);
    fs["Camera.fx"] >> Frame::m_fx;
    fs["Camera.fy"] >> Frame::m_fy;
    fs["Camera.cx"] >> Frame::m_cx;
    fs["Camera.cy"] >> Frame::m_cy;
    fs["DepthMapFactor"] >> Frame::m_depth_factor;
    fs["Camera.width"] >> Frame::m_width;
    fs["Camera.height"] >> Frame::m_height;

    std::string voc_path = "../Voc/ORBvoc.txt";

    LoopClosing loopClosing;
    loopClosing.init(voc_path,true);

    Frame ref_frame;
    Frame* pRef_frame;
    for(int i = 0; i < frameVector.size(); i++)
    {
        std::vector<cv::DMatch> inlier_matches;
        auto& frame = frameVector[i];
        frame.mGlobalIndex = i;
        if(i == 0)
            Tracking::track(frame,NULL,inlier_matches);
        else
        {
            pRef_frame = &frameVector[i-1];
            ref_frame = frameVector[i-1];
            bool success = Tracking::track(frame,&ref_frame,inlier_matches);
            mMap.updateMap(frame,*pRef_frame,inlier_matches,false);
        }
        auto offset_id = generateRandomIds(5,20,2);//offset
        std::vector<int> neighbor_ids;
        for(auto id : offset_id)
        {
            if((i - id ) > 0)
                neighbor_ids.push_back(i - id);
        }
        for(auto id : neighbor_ids)
        {
            if(id < 0)
                continue;
            std::vector<cv::DMatch> matches_cur;
            bool enough_match = Tracking::match(frame,frameVector[id],matches_cur);
            if(enough_match)
            {
                auto& candy_frame = frameVector[id];
                mMap.updateMap(frame,candy_frame,matches_cur,false);
            }

        }

        auto matches = loopClosing.dectectAndGetCandidate(frame.mKepoint_descriptors);
        for(auto& id : matches)
        {
            std::vector<cv::DMatch> matches_cur;
            if(id == i - 1)
                continue;
            bool enough_match = Tracking::match(frame,frameVector[id],matches_cur);
            if(enough_match)
            {
                std::cout<<"candy: "<<id<<"/"<<i<<std::endl;
                auto& candy_frame = frameVector[id];
                mMap.updateMap(frame,candy_frame,matches_cur,true);
            }
        }
    }
}

std::vector<size_t> System::generateRandomIds(int num, int max_id ,int min_id)
{
    std::vector<size_t> ids;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(min_id,max_id);
    for(int i = 0; i < num; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            size_t rand_id = dist(rng);
            auto it = std::find(ids.begin(),ids.end(),rand_id);
            if(it == std::end(ids))
            {
                ids.push_back(rand_id);
                break;
            }
        }
    }

    return ids;
}