//
// Created by jojo on 22.02.20.
//

#include "Frontend.h"
#include "BundleAdjuster.h"
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "Eigen/Geometry"
using namespace BAMapping;


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
        frame.mKepoint_descriptors.push_back(descriptors.row(i).clone());
    }
    frame.mDescriptior = descriptors.clone();
    frame.mpMapPoints.resize(frame.mKeypoints.size(),NULL);
    frame.mKeyPointGlobalIds.resize(frame.mKeypoints.size());
    frame.keyPoint_has_match.resize(frame.mKeypoints.size(),false);
}

void Frontend::ExtractAndMatchFeatures(FrameVector &frameVector, const std::string voc_path)
{
    using namespace cv;
    size_t id = 0;
    for(auto& frame : frameVector)
    {
        frame.mObservations.clear();
        frame.mGlobalIndex = id;
        id++;
    }

    motionModel_.init(frameVector);

    FrameVector out_frameVector;
    Ptr<ORB> orb_detector = ORB::create(500);
    FrameVector::iterator frame_it = frameVector.begin();
    ExtractORB(*frame_it,orb_detector);

    std::cout<<"loading vocabulary..."<<std::endl;
    OrbVocabulary voc(9,3);
//    voc.loadFromTextFile(voc_path);
    voc.load(voc_path);
    std::cout<<"initializing database..."<<std::endl;
    OrbDatabase feature_db(voc,false,0);

    feature_db.add(frame_it->mKepoint_descriptors);

    for(frame_it += 1 ; frame_it != frameVector.end(); frame_it++)
    {
        Frame& frame = *frame_it;
        const Frame& last_frame = *(frame_it -1);
        Frame* pref_frame;
        std::vector<DMatch> matches;
        ExtractORB(frame,orb_detector);

        auto ref_id = detectLoopClosure(feature_db,frame);
        pref_frame = &frameVector[ref_id];

        Frame& ref_frame = *pref_frame;
        matchORB(frame.mDescriptior,ref_frame.mDescriptior,matches);
        if(matches.empty())
        {
            ref_frame = *(frame_it -1);
            matchORB(frame.mDescriptior,ref_frame.mDescriptior,matches);
        }
        removeOutliers(frame,ref_frame,matches,0.08);

//        alignFrames(frame, last_frame.getConstTwc() ,ref_frame, matches); //todo test

        updateMap(frame,*pref_frame, matches);


        //visulize
        Mat img1,img2,img3;
        img1 = imread(frame.getInfraRedImagePath());
        img2 = imread(ref_frame.getInfraRedImagePath());
        drawMatches(img1,frame.mKeypoints,img2,ref_frame.mKeypoints,matches,img3);
        imshow("matches",img3);
        waitKey(1);
    }

}

void Frontend::updateMap(BAMapping::Frame &frame, BAMapping::Frame &ref_frame, const std::vector<cv::DMatch>& matches)
{
    using namespace cv;

    Mat depth_img = imread(frame.getDepthImagePath(),IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img,CV_32F,1.0/1000.0);//todo
    Mat depth_img_ref = imread(ref_frame.getDepthImagePath(),IMREAD_UNCHANGED);
    depth_img_ref.convertTo(depth_img_ref,CV_32F,1.0/1000.0);//todo


    for(const auto& match : matches)
    {
        int i = match.queryIdx;
        int j = match.trainIdx;
        frame.keyPoint_has_match[i] = true;
        ref_frame.keyPoint_has_match[j] = true;
        if(ref_frame.mpMapPoints[j] == nullptr)
        {
            MapPoint mapPoint;
            MapPoint mapPoint_ref;
            bool valid = createNewPoint(mapPoint,frame.mKeypoints[i],frame.mDescriptior.row(i),frame,depth_img);
            bool valid_ref = createNewPoint(mapPoint_ref, ref_frame.mKeypoints[j],ref_frame.mDescriptior.row(j),ref_frame, depth_img_ref);
            if(valid && valid_ref)
            {
                auto dist = mapPoint_ref.pose_ - mapPoint.pose_;
                auto norm = sqrt(dist.dot(dist));
                if(norm > 0.5) //todo
                {
                    std::cout<<"removed 3d point, dist: "<<norm<<std::endl;
                    continue;
                }

                frame.mKeyPointGlobalIds[i] = mMap.mapPoints_.size();
                ref_frame.mKeyPointGlobalIds[j] = mMap.mapPoints_.size();
                mapPoint.id = mMap.mapPoints_.size();
                mapPoint.pose_ = (mapPoint.pose_ + mapPoint_ref.pose_)/2.0; //average todo test
                mMap.addMapPoint(mapPoint);
                auto pMapPoint = std::make_shared<MapPoint>(mapPoint);
                frame.mpMapPoints[i] = pMapPoint;
                ref_frame.mpMapPoints[j] = pMapPoint;
                //generate observations
                auto kp = frame.mKeypoints[i];
                double u = kp.pt.x;
                double v = kp.pt.y;
                double d = depth_img.at<float>(kp.pt);
                frame.mObservations.insert(std::pair<size_t,Eigen::Vector3d>( mapPoint.id,Eigen::Vector3d(u,v,d)));
                kp = ref_frame.mKeypoints[j];
                u = kp.pt.x;
                v = kp.pt.y;
                d = depth_img_ref.at<float>(kp.pt);
                ref_frame.mObservations.insert(std::pair<size_t,Eigen::Vector3d>( mapPoint.id,Eigen::Vector3d(u,v,d)));
            }
            else//triangulate todo
            {

            }
        }
        else
        {
            MapPoint mapPoint;
            MapPoint mapPoint_ref;
            bool valid = createNewPoint(mapPoint,frame.mKeypoints[i],frame.mDescriptior.row(i),frame,depth_img);
            if(valid)
            {
                mapPoint_ref = *ref_frame.mpMapPoints[j];
                auto dist = mapPoint_ref.pose_ - mapPoint.pose_;
                auto norm = sqrt(dist.dot(dist));
                if(norm < 0.5)
                {
                    frame.mpMapPoints[i] = ref_frame.mpMapPoints[j];
                    frame.mKeyPointGlobalIds[i] = ref_frame.mKeyPointGlobalIds[j];
                    auto kp = frame.mKeypoints[i];
                    double u = kp.pt.x;
                    double v = kp.pt.y;
                    double d = depth_img.at<float>(kp.pt);
                    frame.mObservations.insert(std::pair<size_t,Eigen::Vector3d>(frame.mKeyPointGlobalIds[i],Eigen::Vector3d(u,v,d)));
                }
                else
                {
                    std::cout<<"invalid correspondence"<<std::endl;
                }
            }
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
        if(matches[i][0].queryIdx > query.size().height || matches[i][1].queryIdx > query.size().height)
            continue;
        if(matches[i][0].trainIdx > target.size().height || matches[i][1].trainIdx > target.size().height)
            continue;

        if (matches[i][0].distance < matches[i][1].distance * 0.5)
            goodMatches.push_back(matches[i][0]);
    }
}

void Frontend::alignFrames(BAMapping::Frame &frame,const Mat4 last_Twc, const BAMapping::Frame &ref_frame,
                           const std::vector<cv::DMatch> &matches)
{

    auto Tij = motionModel_.getTij(frame.mGlobalIndex,frame.mGlobalIndex - 1);
    auto Tjw = last_Twc.inverse();

    auto Tiw = Tij * Tjw;  //prediction

    if(matches.size()<20)
    {
        std::cout<<"not enough matches, using motion model to propagate"<<std::endl;

        Eigen::Affine3d Tiw_aff;
        Tiw_aff.matrix() = Tiw;
        frame.setFromAffine3d(Tiw_aff);
        return;
    }
    using namespace cv;
    Mat depth_img = imread(frame.getDepthImagePath(),IMREAD_UNCHANGED);
    Mat ref_depth_img = imread(ref_frame.getDepthImagePath(),IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img,CV_32F,1.0/1000.0);
    ref_depth_img.convertTo(ref_depth_img,CV_32F,1.0/1000.0);
    std::vector<Vec3> obs_vec;
    std::vector<Vec3> ref_obs_vec;
    std::vector<Vec3> ref_points;
    for(const auto& match : matches)
    {
        auto kp = frame.mKeypoints[match.queryIdx];
        auto ref_kp = ref_frame.mKeypoints[match.trainIdx];
        double u = kp.pt.x;
        double v = kp.pt.y;
        double d = depth_img.at<float>(kp.pt);
        if(d <= 0.01 || d > 3.0) //todo
        {
            continue;
        }
        double u_ref = ref_kp.pt.x;
        double v_ref = ref_kp.pt.y;
        double d_ref = ref_depth_img.at<float>(ref_kp.pt);
        if(d_ref <= 0.01 || d_ref> 3.0) //todo
        {
            continue;
        }
        obs_vec.push_back(Vec3(u,v,d));
        ref_obs_vec.push_back(Vec3(u_ref,v_ref,d_ref));
    }

    std::cout<<"optimize with matches: "<<obs_vec.size()<<std::endl;
    auto Twc = frame.getConstTwc();
    auto Twc_ref = ref_frame.getConstTwc();

    Twc = Twc_ref;

    BundleAdjuster::optimizePose(Twc,Twc_ref,Vec4(frame.m_fx,frame.m_fy,frame.m_cx,frame.m_cy),obs_vec,ref_obs_vec);

    auto R_opt = Twc.block<3,3>(0,0);
    auto R_mtn = Tiw.inverse().block<3,3>(0,0);  //motion model

    auto t_opt = Twc.block<3,1>(0,3);
    auto t_mtn = Tiw.inverse().block<3,1>(0,3);

    Mat3 R_dist = R_opt * R_mtn.transpose();
    Vec3 R_dist_angleAxis;
    ceres::RotationMatrixToAngleAxis(&R_dist(0),&R_dist_angleAxis(0));

    auto dist = t_opt- t_mtn;
    auto norm = sqrt(dist.dot(dist));
    auto norm_angle = sqrt(R_dist_angleAxis.dot(R_dist_angleAxis));
    if(norm > 0.06 || norm_angle>0.1)
    {
        std::cout<<"result differs from motion model too much, using motion model, norm: "<<norm<<"angle diff: "<<norm_angle<<std::endl;
        Twc = Tiw.inverse();
    }

    Eigen::Affine3d Tcw_aff;
    Tcw_aff.matrix() = Twc.inverse();
    frame.setFromAffine3d(Tcw_aff);
}

void Frontend::ExtractAndCreateDatabase(BAMapping::FrameVector &frameVector, const std::string voc_path, const std::string db_path)
{
    using namespace cv;
    Ptr<ORB> orb_detector = ORB::create(500);

    std::cout<<"loading vocabulary..."<<std::endl;
    OrbVocabulary voc;
    voc.loadFromTextFile(voc_path);
//    voc.load(voc_path);
    std::cout<<"initializing database..."<<std::endl;
    feature_db_.setVocabulary(voc);

    int i = 0;
    for(auto frame_it = frameVector.begin(); frame_it != frameVector.end(); frame_it++)
    {
        ExtractORB(*frame_it,orb_detector);
        std::cout<<"adding to frame database..."<<i<<std::endl;
        feature_db_.add(frame_it->mKepoint_descriptors);
        i++;
    }

    if(!db_path.empty())
    {
        std::cout<<"saving database to: "<<db_path<<std::endl;
        feature_db_.save(db_path);
        std::cout<<"saved database"<<db_path<<std::endl;
    }


}

void Frontend::query(const Frame &frame, DBoW2::QueryResults& ret)
{
    using namespace cv;
    if(frame.mKepoint_descriptors.empty())
    {
        std::cout<<"frame contains no features"<<std::endl;
    }
    feature_db_.query(frame.mKepoint_descriptors,ret,20);
}



void Frontend::removeOutliers(const BAMapping::Frame &frame_query,
                              const BAMapping::Frame &frame_train,
                              std::vector<cv::DMatch> &matches,
                              const double thres)
{

    using namespace cv;
    if(matches.empty())
    {
        std::cout<<"no match"<<std::endl;
        return;
    }
    // Extract location of good matches
    std::vector<Point2f> points1, points2;

    auto keypoints1 = frame_query.mKeypoints;
    auto keypoints2 = frame_train.mKeypoints;
    for( size_t i = 0; i < matches.size(); i++ )
    {
        points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
        points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
    }

    // Find homography
    Mat mask;
    auto h = findHomography( points1, points2, mask,RANSAC);
    std::vector<cv::DMatch> good_matches;
    for(int i = 0; i < matches.size(); i++)
    {
        int inlier = mask.at<uchar>(i);
//        std::cout<<inlier<<std::endl;
        if(inlier)
        {
            good_matches.push_back(matches[i]);
        }
    }
    int removed = matches.size() - good_matches.size();
    matches.clear();
    matches = good_matches;
    printf("removed : %d\n",removed);
}


void Frontend::triangulateKeyPoints(const BAMapping::Frame &frame, const BAMapping::Frame &ref_frame,
                                    const std::vector <cv::DMatch> &matches, vector <MapPoint> &mapPoints)
{
    using namespace cv;
    std::vector<Point2f> cam0pnts;
    std::vector<Point2f> cam1pnts;
    Mat pnts3D(4,cam0pnts.size(),CV_64F);
    for(const auto& match : matches)
    {
        int i = match.queryIdx;
        int j = match.trainIdx;
        if(ref_frame.mpMapPoints[j] == NULL)
        {
            cam0pnts.emplace_back(frame.mKeypoints[i].pt);
            cam1pnts.emplace_back(ref_frame.mKeypoints[j].pt);
        }
    }
    if(cam0pnts.empty())
    {
        std::cout<<"no point to be triangulated"<<std::endl;
        return;
    }
    Mat cam0pose(3,4,CV_64F);
    Mat cam1pose(3,4,CV_64F);
    Eigen::Matrix3f A;
    Eigen::Matrix<float,3,4> projectM;
    A<< frame.m_fx,0,frame.m_cx,
        0,frame.m_fy,frame.m_cy,
        0,0,1;
    projectM.block<3,3>(0,0) = frame.getConstTcw().block<3,3>(0,0).cast<float>();
    projectM.block<3,1>(0,3) = frame.getConstTcw().block<3,1>(0,3).cast<float>();
    projectM = A * projectM;
//    std::cout<<projectM<<std::endl;

    cv::eigen2cv(projectM,cam0pose);
//    std::cout<<"cam0: "<<cam0pose<<std::endl;
    projectM.block<3,3>(0,0) = ref_frame.getConstTcw().block<3,3>(0,0).cast<float>();
    projectM.block<3,1>(0,3) = ref_frame.getConstTcw().block<3,1>(0,3).cast<float>();
    projectM = A * projectM;
//    std::cout<<projectM<<std::endl;

    cv::eigen2cv(projectM,cam1pose);
//    std::cout<<"cam1: "<<cam1pose<<std::endl;

    cv::triangulatePoints(cam0pose,cam1pose,cam0pnts,cam1pnts,pnts3D);

    for(int i = 0; i < pnts3D.cols; i++)
    {

        Vec4 point_pose;
//        point_pose[0] = pnts3D.at<double>(0,i);
//        point_pose[1] = pnts3D.at<double>(1,i);
//        point_pose[2] = pnts3D.at<double>(2,i);
        cv::cv2eigen(pnts3D.col(i),point_pose);
//        std::cout<<"mat:"<<pnts3D.col(i)<<std::endl;
//        std::cout<<point_pose<<std::endl;

        MapPoint p(point_pose.block<3,1>(0,0));
        mapPoints.push_back(p);
    }
}

void Frontend::LoadDataBase(const std::string db_path)
{
    std::cout<<"loading dababase from"<<db_path<<std::endl;
    OrbDatabase db(db_path);
    std::cout<<"loaded dababase"<<db<<std::endl;
//    feature_db_ = db;
}

int Frontend::detectLoopClosure(OrbDatabase& db, const BAMapping::Frame &frame)
{
    using namespace cv;
    DBoW2::QueryResults ret;
    db.query(frame.mKepoint_descriptors,ret,3);
    db.add(frame.mKepoint_descriptors);

    if(!ret.empty())
    {
        auto best_id = ret[0].Id;
        std::sort(ret.begin(), ret.end(),
                  [](DBoW2::Result a, DBoW2::Result b)
                  {
                      return a.Id < b.Id;
                  });
        std::vector<DMatch> matches;
        for(auto r : ret)
        {
            if(r.Id - best_id > 50 && r.Score>0.1)
            {
                return r.Id;
            }
        }
        return best_id;
    }
    return -1;
}