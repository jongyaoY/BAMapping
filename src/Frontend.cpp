//
// Created by jojo on 22.02.20.
//

#include "Frontend.h"
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "Eigen/Geometry"
using namespace BAMapping;


void Frontend::ExtractAllFeaturesAndTrainMatcher(FrameVector &frameVector)
{
    using namespace cv;
    Ptr<ORB> orb_detector = ORB::create(300);
    for(auto& frame : frameVector)
    {
        Mat ir_img = imread(frame.getInfraRedImagePath(),IMREAD_GRAYSCALE);
        std::vector<KeyPoint> keypoints;
        Mat descriptors;
        orb_detector->detect( ir_img, keypoints );
        orb_detector->compute(ir_img,keypoints,descriptors);
        frame.mDescriptior = descriptors;
        for(int i = 0; i < keypoints.size(); i++)
        {
            frame.mKeypoints.push_back(keypoints[i]);
            frame.mKepoint_descriptors.push_back(descriptors.row(i));
        }
//        std::cout<<keypoints.size()<<" "<<descriptor.size()<<std::endl;

        Mat img_keypoints;
        drawKeypoints( ir_img, keypoints, img_keypoints );
        imshow("orb key points",img_keypoints);
        waitKey(1);
        mMap.matcher_->add(descriptors);
    }
    std::cout<<"training ..."<<std::endl;
    mMap.matcher_->train();
    std::cout<<"finished"<<std::endl;
}

void Frontend::ExtractFeatures(FrameVector &frameVector)
{
    using namespace cv;
    using namespace cv::xfeatures2d;

    Ptr<ORB> orb_detector = ORB::create(300);
    Ptr<SURF> surf_detector = SURF::create(400);
    Ptr<SIFT> sift_detector = SIFT::create(100);
    namedWindow("orb key points");
    for(auto& frame : frameVector)
    {
        Mat ir_img = imread(frame.getInfraRedImagePath(),IMREAD_GRAYSCALE);
        std::vector<KeyPoint> keypoints;
        Mat descriptors;
//        orb_detector->detect( ir_img, keypoints );
//        orb_detector->compute(ir_img,keypoints,descriptors);
        sift_detector->detectAndCompute( ir_img, noArray(), keypoints, descriptors);
        frame.mDescriptior = descriptors;
        for(int i = 0; i < keypoints.size(); i++)
        {
            frame.mKeypoints.push_back(keypoints[i]);
            frame.mKepoint_descriptors.push_back(descriptors.row(i));
        }
//        std::cout<<keypoints.size()<<" "<<descriptor.size()<<std::endl;

        Mat img_keypoints;
        drawKeypoints( ir_img, keypoints, img_keypoints );
        imshow("orb key points",img_keypoints);
        waitKey(1);

    }

}

void Frontend::ExtractORB(Frame& frame,cv::Ptr<cv::ORB> orb_detector)
{
    using namespace cv;
    Mat ir_img = imread(frame.getInfraRedImagePath(),IMREAD_GRAYSCALE);
    std::vector<KeyPoint> keypoints;
    Mat descriptors;
    orb_detector->detect( ir_img, keypoints );
    orb_detector->compute(ir_img,keypoints,descriptors);
    for(int i = 0; i < keypoints.size(); i++)
    {
        frame.mKeypoints.push_back(keypoints[i]);
    }
    frame.mDescriptior = descriptors.clone();
}

void Frontend::ExtractAndMatchFeatures(FrameVector &frameVector)
{
    using namespace cv;
    Ptr<ORB> orb_detector = ORB::create(300);
    FrameVector::iterator frame_it = frameVector.begin();
    ExtractORB(*frame_it,orb_detector);
    initMap(*frame_it);

    Frame ref_frame = *frame_it;
    for(frame_it += 1 ; frame_it != frameVector.end(); frame_it++)
    {
        Frame& frame = *frame_it;
        std::vector<DMatch> matches;
        ExtractORB(frame,orb_detector);
        matchORB(frame.mDescriptior,ref_frame.mDescriptior,matches);
        updateMap(frame,ref_frame,matches);

        //visulize
        Mat img1,img2,img3;
        img1 = imread(frame.getInfraRedImagePath());
        img2 = imread(ref_frame.getInfraRedImagePath());
        drawMatches(img1,frame.mKeypoints,img2,ref_frame.mKeypoints,matches,img3);
        imshow("matches",img3);
        waitKey(1);

        ref_frame = frame;
    }

}

void Frontend::initMap(BAMapping::Frame &first_frame)
{
    using namespace cv;
    first_frame.keyPoint_has_match.resize(first_frame.mKeypoints.size(),false);
    first_frame.mKeyPointGlobalIds.resize(first_frame.mKeypoints.size());
    Mat depth_img = imread(first_frame.getDepthImagePath(),IMREAD_UNCHANGED);

    depth_img.convertTo(depth_img,CV_32F);
    depth_img /= 1000.0; //todo

    for(int i = 0; i < first_frame.mKeypoints.size(); i++)
    {
        auto keyPoint = first_frame.mKeypoints[i];
        MapPoint mapPoint;
        bool valid = createNewPoint(mapPoint,keyPoint,first_frame.mDescriptior.row(i),first_frame,depth_img);
        if(valid)
        {
            first_frame.mKeyPointGlobalIds[i] = mMap.mapPoints_.size();
            mMap.addMapPoint(mapPoint);
//            new_mapPoints.push_back(mapPoint);
        }
        else
        {
            first_frame.mKeyPointGlobalIds[i] = -1;
        }

    }

}

void Frontend::updateMap(BAMapping::Frame &frame, const BAMapping::Frame &ref_frame, std::vector<cv::DMatch> matches)
{
    using namespace cv;
    frame.keyPoint_has_match.resize(frame.mKeypoints.size(),false);
    frame.mKeyPointGlobalIds.resize(frame.mKeypoints.size());
    Mat depth_img = imread(frame.getDepthImagePath(),IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img,CV_32F);
    depth_img /= 1000.0; //todo

    for(const auto& match : matches)
    {
        size_t match_global_id = ref_frame.mKeyPointGlobalIds[match.trainIdx];
        frame.keyPoint_has_match[match.queryIdx] = true;
        frame.mKeyPointGlobalIds[match.queryIdx] = match_global_id;
    }

    for(int i = 0; i < frame.mKeypoints.size(); i++)
    {
        bool is_match = frame.keyPoint_has_match[i];
        auto keyPoint = frame.mKeypoints[i];
        if(!is_match)
        {
            MapPoint mapPoint;
            bool valid = createNewPoint(mapPoint,keyPoint,frame.mDescriptior.row(i),frame,depth_img);
            if(valid)
            {
                frame.mKeyPointGlobalIds[i] = mMap.mapPoints_.size();
                mMap.addMapPoint(mapPoint);
            }
            else
            {
                frame.mKeyPointGlobalIds[i] = -1;
            }
        }
    }
}

void Frontend::CreatePointCloud(FrameVector &frameVector)
{
    using namespace cv;
    Frame ref_frame;

    for(auto& frame : frameVector)
    {
        Mat depth_img = imread(frame.getDepthImagePath(),IMREAD_UNCHANGED);
        depth_img.convertTo(depth_img,CV_32F);
        depth_img /= 1000.0; //todo
        if(!mMap.mapPoints_.empty())
        {

            std::vector<DMatch> matches;
//            bf.match(frame.mDescriptior,ref_frame.mDescriptior,matches);
//            mMap.matchFeatures(frame.mDescriptior,matches);
//            std::sort(matches.begin(),matches.end(),compare_distance);
            matchFeatures(frame.mDescriptior,ref_frame.mDescriptior,matches);
            Mat img1,img2,img3;
            img1 = imread(frame.getInfraRedImagePath());
            img2 = imread(ref_frame.getInfraRedImagePath());
            drawMatches(img1,frame.mKeypoints,img2,ref_frame.mKeypoints,matches,img3);
            imshow("matches",img3);
            waitKey(1);

            ref_frame = frame;

            std::cout<<"matches between frames: "<<matches.size()<<std::endl;

            std::vector<bool> is_match(frame.mKeypoints.size(),false);
//            auto keyPoints = frame.mKeypoints;
            for(const auto& match : matches)
            {
                is_match[match.queryIdx] = true;
//                keyPoints.erase(keyPoints.begin() + match.queryIdx);
            }
            std::vector<MapPoint> mapPoints;
            Mat query_unmatch;
            std::vector<size_t> unmatch_id;
            for(int i = 0; i < frame.mKeypoints.size(); i++)
            {
                if(is_match[i])
                {

                }
                else
                {
                    query_unmatch.push_back(frame.mDescriptior.row(i));
                    unmatch_id.push_back(i);

                }

            }
            //find matches in map
//            matches.clear();
//            mMap.matchFeatures(query_unmatch,matches);
//            std::cout<<"matches in map: "<<matches.size()<<std::endl;
//
//            for(const auto& match : matches)
//            {
//                size_t match_id = unmatch_id[match.queryIdx];
//                is_match[match_id] = true;
//            }
//
//            for(int i = 0; i < frame.mKeypoints.size(); i++)
//            {
//                if(is_match[i])
//                {
//
//                }
//                else
//                {
//                    const auto& point = frame.mKeypoints[i];
//                    MapPoint p;
//                    createNewPoint(p,point,frame.mDescriptior.row(i),frame,depth_img,3.0);
//                    mapPoints.push_back(p);
//                }
//            }
//
//            mMap.addPoints(mapPoints);

//            std::cout<<"create new points: "<<mapPoints.size()<<std::endl;




//            if(matches.size()/frame.mKeypoints.size()<0.5 || (frame.mGlobalIndex - ref_frame.mGlobalIndex) > 10)
//            {
//
//            }

        }
        else //initialize map
        {
            std::vector<MapPoint> mapPoints;
            for(int i = 0; i < frame.mKeypoints.size(); i++)
            {
                const auto& point = frame.mKeypoints[i];

                MapPoint p;
                createNewPoint(p,point,frame.mDescriptior.row(i),frame,depth_img,3.0);
                mapPoints.push_back(p);
            }
            mMap.init(frame.mDescriptior,mapPoints);
            ref_frame = frame;
        }
    }

}

bool Frontend::createNewPoint(MapPoint& mapPoint, const cv::KeyPoint& point, const cv::Mat& descriptor, const Frame& frame, const cv::Mat& depth_img,const double depth_thres)
{
    Vec3 pose;
    double u = point.pt.x;
    double v = point.pt.y;
    double d = depth_img.at<float>(point.pt);

    if(d <= 0.01 || d > depth_thres)
        return false;

    pose[0] = (u-frame.m_cx)*d/frame.m_fx;
    pose[1] = (v-frame.m_cy)*d/frame.m_fy;
    pose[2] = d;

    Eigen::Affine3d Twc;
    Twc.matrix() = frame.getConstTwc();
    pose = Twc * pose;

    mapPoint = MapPoint(pose);
    mapPoint.descritor_ = descriptor.clone();
    return true;
}

void Frontend::matchORB(const cv::Mat &query, const cv::Mat &target, std::vector<cv::DMatch> &goodMatches)
{
    using namespace cv;
    std::vector<std::vector<cv::DMatch>> matches;
//    cv::Ptr<cv::FlannBasedMatcher> matcher = new cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    Ptr<BFMatcher> matcher = BFMatcher::create(NORM_HAMMING,false);
    matcher->knnMatch(query, target, matches, 2);

    for (unsigned int i = 0; i < matches.size(); ++i)
    {
        if(matches[i].empty())
            continue;

        if (matches[i][0].distance < matches[i][1].distance * 0.75)
            goodMatches.push_back(matches[i][0]);
    }
}
void Frontend::query(const Frame &frame,std::vector<cv::DMatch> &goodMatches)
{
    std::vector<std::vector<cv::DMatch>> matches;

    mMap.matcher_->knnMatch(frame.mDescriptior,mMap.matcher_->getTrainDescriptors(),matches,3);
    for (unsigned int i = 0; i < matches.size(); ++i)
    {
        if(matches[i].empty())
            continue;

        if (matches[i][1].distance < matches[i][2].distance * 0.75)
            goodMatches.push_back(matches[i][1]);
    }

}
void Frontend::matchFeatures(const cv::Mat &query, const cv::Mat &target, std::vector<cv::DMatch> &goodMatches)
{
    using namespace cv;
    std::vector<std::vector<cv::DMatch>> matches;

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
//    cv::Ptr<cv::BFMatcher> matcher = new BFMatcher(NORM_HAMMING, true);
    // Find 2 best matches for each descriptor to make later the second neighbor test.
    matcher->knnMatch(query, target, matches, 2);
    // Second neighbor ratio test.
    for (unsigned int i = 0; i < matches.size(); ++i)
    {
        if(matches[i].empty())
            continue;

        if (matches[i][0].distance < matches[i][1].distance * 0.75)
            goodMatches.push_back(matches[i][0]);
    }
}

void Frontend::ExtractAllFeaturesAndCreateDB(FrameVector &frameVector,std::string db_name)
{
    using namespace cv;
    using namespace DBoW2;
    Ptr<ORB> orb_detector = ORB::create(300);
    OrbDatabase db(false,0);

    for(auto& frame : frameVector)
    {
        Mat ir_img = imread(frame.getInfraRedImagePath(),IMREAD_GRAYSCALE);
        std::vector<KeyPoint> keypoints;
        Mat descriptors;
        orb_detector->detect( ir_img, keypoints );
        orb_detector->compute(ir_img,keypoints,descriptors);
        frame.mDescriptior = descriptors;
        for(int i = 0; i < keypoints.size(); i++)
        {
            frame.mKeypoints.push_back(keypoints[i]);
            frame.mKepoint_descriptors.push_back(descriptors.row(i));
        }
//        std::cout<<keypoints.size()<<" "<<descriptor.size()<<std::endl;
        db.add(frame.mKepoint_descriptors);
        Mat img_keypoints;
        drawKeypoints( ir_img, keypoints, img_keypoints );
        imshow("orb key points",img_keypoints);
        waitKey(1);
    }

    std::cout << "Saving database to"<<db_name<<" ..." << std::endl;
    db.save(db_name);
    std::cout<<"done"<<std::endl;
}
