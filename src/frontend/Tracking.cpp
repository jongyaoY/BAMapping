//
// Created by jojo on 25.03.20.
//

#include "Tracking.h"
#include "Alignment3D_ransac.h"
#include "../GeometryMethods.h"

#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
using namespace BAMapping::FrontEnd;

bool Tracking::track(Frame &frame, const Frame* pRef_frame, std::vector<cv::DMatch> &inlier_matches)
{
    using namespace cv;
    ExtractORB(frame);
    if(pRef_frame == NULL)
    {
        Eigen::Affine3d Tcw;
        frame.Tcw_ = Mat4::Identity();
        Tcw.matrix() = frame.Tcw_;
        frame.setFromAffine3d(Tcw);

        return true;
    }
    auto ref_frame = *pRef_frame;
    std::vector<DMatch> matches;
    std::vector<DMatch> matches_inliers;

    MatchORB(frame.mDescriptior,ref_frame.mDescriptior,matches);

    std::vector<Vec3> points;
    std::vector<Vec3> ref_points;
    std::vector<size_t> inliers;
    Mat4 Tts;
    auto matches_valid = GetSparsePointClouds(frame,ref_frame,matches,points,ref_points);

    bool success = Alignment3D_ransac::Align(points,ref_points,Tts,inliers,Alignment3D_ransac::Params());

    if(success)
    {
        for(auto id : inliers)
        {
            matches_inliers.push_back(matches_valid[id]);
            inlier_matches.push_back(matches_valid[id]);
        }
        Mat img1,img2,img3;
        if(!frame.getInfraRedImagePath().empty())
        {
            img1 = imread(frame.getInfraRedImagePath());
            img2 = imread(ref_frame.getInfraRedImagePath());
        }
        else
        {
            img1 = imread(frame.getRGBImagePath());
            img2 = imread(ref_frame.getRGBImagePath());
        }
        if(!img1.empty()&&!img2.empty())
        {
            drawMatches(img1,frame.mKeypoints,img2,ref_frame.mKeypoints,matches_inliers,img3);
            imshow("matches",img3);
            waitKey(1);
        }
        else
        {
            cv::destroyWindow("matches");
        }



        Eigen::Affine3d Tcw;
        frame.Tcw_ = Tts.inverse() * ref_frame.Tcw_;
        Tcw.matrix() = frame.Tcw_;
        frame.setFromAffine3d(Tcw);


        return true;
    }
    else
    {
        Eigen::Affine3d Tcw;
        frame.Tcw_ = ref_frame.Tcw_;
        Tcw.matrix() = frame.Tcw_;
        frame.setFromAffine3d(Tcw);
        return false;
    }


}

bool Tracking::match(const BAMapping::Frame &frame, const BAMapping::Frame& ref_frame,
                     std::vector<cv::DMatch> &inlier_matches)
{
    using namespace cv;
    std::vector<DMatch> matches;
//    std::vector<DMatch> matches_inliers;
    inlier_matches.clear();

    MatchORB(frame.mDescriptior,ref_frame.mDescriptior,matches);
    std::vector<Vec3> points;
    std::vector<Vec3> ref_points;
    std::vector<size_t> inliers;
    Mat4 Tts;
    auto matches_valid = GetSparsePointClouds(frame,ref_frame,matches,points,ref_points);

    Alignment3D_ransac::Align(points,ref_points,Tts,inliers,Alignment3D_ransac::Params());
    for(auto id : inliers)
    {
        inlier_matches.push_back(matches_valid[id]);
    }
    float score = (float) inlier_matches.size() / (float) frame.mKeypoints.size();
    if(inlier_matches.size()<5) //todo score < 0.1 ||
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool Tracking::extractAndMatch(BAMapping::Frame &frame, const BAMapping::Frame *pRef_frame,
                               std::vector<cv::DMatch> &inlier_matches)
{
    using namespace cv;
    ExtractORB(frame);
    if(pRef_frame == NULL)
    {
        Eigen::Affine3d Tcw;
        frame.Tcw_ = Mat4::Identity();
        Tcw.matrix() = frame.Tcw_;
        frame.setFromAffine3d(Tcw);

        return true;
    }
    auto ref_frame = *pRef_frame;
    std::vector<DMatch> matches;
    std::vector<DMatch> matches_inliers;

    MatchORB(frame.mDescriptior,ref_frame.mDescriptior,matches);

    std::vector<Vec3> points;
    std::vector<Vec3> ref_points;
    std::vector<size_t> inliers;
    Mat4 Tts;
    auto matches_valid = GetSparsePointClouds(frame,ref_frame,matches,points,ref_points);

    bool success = Alignment3D_ransac::Align(points,ref_points,Tts,inliers,Alignment3D_ransac::Params());

    if(!success)
    {
        return false;
    }

    for(auto id : inliers)
    {
        matches_inliers.push_back(matches_valid[id]);
        inlier_matches.push_back(matches_valid[id]);
    }
    Mat img1,img2,img3;
    if(!frame.getInfraRedImagePath().empty())
    {
        img1 = imread(frame.getInfraRedImagePath());
        img2 = imread(ref_frame.getInfraRedImagePath());
    }
    else
    {
        img1 = imread(frame.getRGBImagePath());
        img2 = imread(ref_frame.getRGBImagePath());
    }
    if(!img1.empty()&&!img2.empty())
    {
        drawMatches(img1,frame.mKeypoints,img2,ref_frame.mKeypoints,matches_inliers,img3);
        imshow("matches",img3);
        waitKey(1);
    }
    else
    {
        cv::destroyWindow("matches");
    }
    return true;
}

void Tracking::ExtractORB(BAMapping::Frame &frame)
{
    using namespace cv;
    Mat img;
    if(!frame.getInfraRedImagePath().empty())
        img = imread(frame.getInfraRedImagePath(),IMREAD_GRAYSCALE);
    else
        img = imread(frame.getRGBImagePath(),IMREAD_GRAYSCALE);

    Mat depth_img = imread(frame.getDepthImagePath(),IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img,CV_32F,1.0/Frame::m_depth_factor);

    Ptr<ORB> orb_detector = ORB::create(500);;
    std::vector<KeyPoint> keypoints;

    Mat descriptors;
    orb_detector->detect( img, keypoints );
    orb_detector->compute(img,keypoints,descriptors);
    for(int i = 0; i < keypoints.size(); i++)
    {
        frame.mKeypoints.push_back(keypoints[i]);
        frame.mKepoint_descriptors.push_back(descriptors.row(i).clone());

        double d = depth_img.at<float>(keypoints[i].pt);
        frame.mKeyPointsDepth.push_back(d);
    }
    frame.mDescriptior = descriptors.clone();
    frame.mpMapPoints.resize(frame.mKeypoints.size(), nullptr);
    frame.mKeyPointGlobalIds.resize(frame.mKeypoints.size());
    frame.keyPoint_has_match.resize(frame.mKeypoints.size(),false);
}

std::vector<cv::DMatch> Tracking::GetSparsePointClouds(const BAMapping::Frame &frame, const BAMapping::Frame &ref_frame,
                                    const std::vector<cv::DMatch> &matches,
                                    std::vector<Vec3>& frame_pt,
                                    std::vector<Vec3>& ref_frame_pt)
{
    std::vector<cv::DMatch> matches_valid;
    if(matches.empty())
    {
        return matches_valid;
    }
    frame_pt.clear();
    ref_frame_pt.clear();
    struct
    {
    public:
        Vec3 operator ()(Vec3 obs,Vec4 intrinsics)
        {
            auto fx = intrinsics[0];
            auto fy = intrinsics[1];
            auto cx = intrinsics[2];
            auto cy = intrinsics[3];
            auto u = obs[0];
            auto v = obs[1];
            auto d = obs[2];
            return Vec3((u-cx)*d/fx,(v-cy)*d/fy,d);
        }
    }project;

    for(auto& match : matches)
    {
        auto kp = frame.mKeypoints[match.queryIdx];
        auto kp_d = frame.mKeyPointsDepth[match.queryIdx];
        auto ref_kp = ref_frame.mKeypoints[match.trainIdx];
        auto ref_kp_d = ref_frame.mKeyPointsDepth[match.trainIdx];

        if(kp_d < 0.001 || kp_d > 10.0 || ref_kp_d < 0.001 || ref_kp_d > 10.0)//todo
        {
            continue;
        }
        auto point = project(Vec3(kp.pt.x,kp.pt.y,kp_d),Vec4(Frame::m_fx,Frame::m_fy,Frame::m_cx,Frame::m_cy));
        auto ref_point = project(Vec3(ref_kp.pt.x,ref_kp.pt.y,ref_kp_d),Vec4(Frame::m_fx,Frame::m_fy,Frame::m_cx,Frame::m_cy));

        frame_pt.push_back(point);
        ref_frame_pt.push_back(ref_point);
        matches_valid.push_back(match);
    }
    return matches_valid;
}

void Tracking::MatchORB(const cv::Mat &query, const cv::Mat &target, std::vector<cv::DMatch> &goodMatches)
{
    using namespace cv;
    std::vector<std::vector<cv::DMatch>> matches;
    cv::Ptr<cv::FlannBasedMatcher> matcher = new cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
//    Ptr<BFMatcher> matcher = BFMatcher::create(NORM_HAMMING,false);
    matcher->knnMatch(query, target, matches, 2);

    for (unsigned int i = 0; i < matches.size(); ++i)
    {
        if(matches[i].empty())
            continue;
        if(matches[i][0].queryIdx > query.size().height || matches[i][1].queryIdx > query.size().height)
            continue;
        if(matches[i][0].trainIdx > target.size().height || matches[i][1].trainIdx > target.size().height)
            continue;

        if (matches[i][0].distance < matches[i][1].distance * 0.8)
            goodMatches.push_back(matches[i][0]);
    }
}


