//
// Created by jojo on 25.03.20.
//

#include <iostream>
#include "Mapping.h"

using namespace BAMapping::FrontEnd;

void Mapping::updateMap(Frame &frame, Frame &ref_frame, std::vector<cv::DMatch> matches, bool loopClosure)
{
    struct
    {
    public:
        bool operator ()(const Vec3 obs,const Vec4 intrinsics, Vec3& point, Mat4 Twc)
        {
            auto fx = intrinsics[0];
            auto fy = intrinsics[1];
            auto cx = intrinsics[2];
            auto cy = intrinsics[3];
            auto u = obs[0];
            auto v = obs[1];
            auto d = obs[2];
            if(d < 0.001 || d > 4.0)
                return false;
            point = Vec3((u-cx)*d/fx,(v-cy)*d/fy,d);
//            Mat4 Tcw = Twc.inverse();
            point = Twc.block<3,3>(0,0) * point + Twc.block<3,1>(0,3);
            return true;
        }
    }project;

    for(const auto& match : matches)
    {
        int i = match.queryIdx;
        int j = match.trainIdx;
        frame.keyPoint_has_match[i] = true;
        ref_frame.keyPoint_has_match[j] = true;

        auto kp = frame.mKeypoints[i];
        auto kp_d = frame.mKeyPointsDepth[i];
        auto ref_kp = ref_frame.mKeypoints[j];
        auto ref_kp_d = ref_frame.mKeyPointsDepth[j];
        if(frame.mpMapPoints[i] == nullptr)
        {
            if(ref_frame.mpMapPoints[j] == nullptr)
            {
                MapPoint point;
                MapPoint point_ref;
                {

                    bool valid_point = project(Vec3(kp.pt.x,kp.pt.y,kp_d),
                                               Vec4(Frame::m_fx,Frame::m_fy,Frame::m_cx,Frame::m_cy),
                                               point.pose_,
                                               frame.Tcw_.inverse());
                    bool valid_ref_point = project(Vec3(ref_kp.pt.x,ref_kp.pt.y,ref_kp_d),
                                                   Vec4(Frame::m_fx,Frame::m_fy,Frame::m_cx,Frame::m_cy),
                                                   point_ref.pose_,
                                                   ref_frame.Tcw_.inverse());
                    if(!valid_ref_point || !valid_point)
                    {
                        continue;
                    }

                    frame.mKeyPointGlobalIds[i] = m_map_points.size();
                    ref_frame.mKeyPointGlobalIds[j] = m_map_points.size();
                    point.id = m_map_points.size();
                    point.pose_ = (point.pose_ + point_ref.pose_)/2.0;

                    m_map_points.push_back(point);
                    auto pMapPoint = std::make_shared<MapPoint>(point);
                    frame.mpMapPoints[i] = pMapPoint;
                    ref_frame.mpMapPoints[j] = pMapPoint;
                    //generate observations

                    frame.mObservations.insert(std::pair<size_t,Eigen::Vector3d>( point.id,Eigen::Vector3d(kp.pt.x,kp.pt.y,kp_d)));
                    ref_frame.mObservations.insert(std::pair<size_t,Eigen::Vector3d>( point.id,Eigen::Vector3d(ref_kp.pt.x,ref_kp.pt.y,ref_kp_d)));
                }
            }
            else
            {
                MapPoint mapPoint;
                MapPoint mapPoint_ref;
                bool valid_point = project(Vec3(kp.pt.x,kp.pt.y,kp_d),
                                           Vec4(Frame::m_fx,Frame::m_fy,Frame::m_cx,Frame::m_cy),
                                           mapPoint.pose_,
                                           frame.Tcw_.inverse());
                if(valid_point)
                {
                    mapPoint_ref = *ref_frame.mpMapPoints[j];

                    {
                        frame.mpMapPoints[i] = ref_frame.mpMapPoints[j];
                        frame.mKeyPointGlobalIds[i] = ref_frame.mKeyPointGlobalIds[j];
                        frame.mObservations.insert(std::pair<size_t,Eigen::Vector3d>( mapPoint_ref.id,Eigen::Vector3d(kp.pt.x,kp.pt.y,kp_d)));
                    }
                }
            }
        }
        else
        {
            if(ref_frame.mpMapPoints[j] == nullptr)
            {
                MapPoint mapPoint;
                MapPoint mapPoint_ref;
                bool valid_ref_point = project(Vec3(kp.pt.x,kp.pt.y,kp_d),
                                           Vec4(Frame::m_fx,Frame::m_fy,Frame::m_cx,Frame::m_cy),
                                           mapPoint_ref.pose_,
                                           frame.Tcw_.inverse());
                if(valid_ref_point)
                {
                    mapPoint = *frame.mpMapPoints[i];
                    ref_frame.mpMapPoints[j] = frame.mpMapPoints[i];
                    ref_frame.mKeyPointGlobalIds[j] = frame.mKeyPointGlobalIds[i];
                    ref_frame.mObservations.insert(std::pair<size_t,Eigen::Vector3d>( mapPoint.id,Eigen::Vector3d(ref_kp.pt.x,ref_kp.pt.y,ref_kp_d)));
                }
            }
            else
            {
                if(loopClosure)
                {

                    auto point = frame.mpMapPoints[i];
                    auto ref_point = ref_frame.mpMapPoints[j];
                    if(point->id != ref_point->id)
                    {
                        frame.mpMapPoints[i] = ref_frame.mpMapPoints[j];
                        frame.mKeyPointGlobalIds[i] = ref_frame.mKeyPointGlobalIds[j];
                        frame.mObservations[point->id] = Eigen::Vector3d(kp.pt.x,kp.pt.y,kp_d);
                    }
                }
            }
        }
    }
}