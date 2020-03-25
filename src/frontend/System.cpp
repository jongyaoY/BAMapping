//
// Created by jojo on 25.03.20.
//

#include "System.h"
#include <opencv2/core/persistence.hpp>

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

    Frame ref_frame;
    for(int i = 0; i < frameVector.size(); i++)
    {
        std::vector<cv::DMatch> inlier_matches;
        auto& frame = frameVector[i];
        if(i == 0)
            Tracking::track(frame,NULL,inlier_matches);
        else
        {
            ref_frame = frameVector[i-1];
            bool success = Tracking::track(frame,&ref_frame,inlier_matches);
            mMap.updateMap(frame,ref_frame,inlier_matches);
        }
    }
}