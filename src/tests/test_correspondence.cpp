//
// Created by jojo on 10.02.20.
//

#include "../io/Reader.h"
#include "../Frontend.h"
#include "../io/Writer.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "DBoW2.h"
int main(int argc, char** argv)
{
    using namespace BAMapping;
    std::string dataset_path = "../dataset/ITE_Office/";
    std::string voc_path = dataset_path + "ORBvoc.yml.gz";
    std::string obs_path = dataset_path + "observations.txt";
    std::string point_path = dataset_path + "points.txt";
    std::string cam_file = dataset_path + argv[1];
    std::string cam_out_file = dataset_path + "cameras_out.txt";
    std::string config_file = dataset_path + "ITE.yaml";
    auto frameVec = io::Reader::readITEFrames(cam_file.c_str(),
                                              (dataset_path + "observations.txt").c_str(),
                                              dataset_path.c_str(),1);

    Frontend frontend;
    frontend.ExtractAndMatchFeatures(frameVec,voc_path);

//    for(auto& frame : frameVec)
//    {
//        frame.generateObservations();
//    }

    PointVector obs_pointVector;

    for(auto frame : frameVec)
    {
        double fx = frame.m_fx;
        double fy = frame.m_fy;
        double cx = frame.m_cx;
        double cy = frame.m_cy;
        Eigen::Affine3d Twc;
        Twc.matrix() = frame.getConstTwc();
        for(auto id_obs : frame.mObservations)
        {
            Eigen::Vector3d p_obs;
            Eigen::Vector3d p_world;
            auto point_id = id_obs.first;
            auto obs = id_obs.second;

            p_obs[0] = (obs[0]-cx)*obs[2]/fx;
            p_obs[1] = (obs[1]-cy)*obs[2]/fy;
            p_obs[2] = obs[2];

            p_obs = Twc * p_obs;

            Point p(p_obs);
            obs_pointVector.push_back(p);

        }
    }
    PointVector pointVector;
    for(auto point : frontend.mMap.mapPoints_)
    {
        Point p(point.pose_);
        pointVector.push_back(p);
    }
//    FrameMethods::filterObservations(frameVec,pointVector,config_file.c_str());
    Writer::writePoses(frameVec,cam_out_file.c_str());
    Writer::writeObservations(frameVec,obs_path.c_str());
    Writer::writePoints(frontend.mMap,point_path.c_str());
    Viewer viewer;
//    viewer.setMap(&frontend.mMap);
//    viewer.visualizeMap();
    viewer.setPoints(obs_pointVector);
    viewer.setFrames(frameVec);
    viewer.visualize();
}
//
//
//
//{
//    using namespace BAMapping;
//    std::string dataset_path = "../dataset/ITE_Long/";
//    std::string config_file = dataset_path + "ITE.yaml";
//
//    auto frameVec = io::Reader::readITEFrames((dataset_path + "cameras.txt").c_str(),
//                                              (dataset_path + "observations.txt").c_str(),
//                                              dataset_path.c_str(),1);
//    auto ref_pointVec = io::Reader::readPoints((dataset_path + "points.txt").c_str());
//
//    int sum = 0;
//    int min = 100;
//    for(auto frame : frameVec)
//    {
//        sum += frame.getObservations().size();
//        if(frame.getObservations().size() < min && !frame.getObservations().empty())
//            min = frame.getObservations().size();
//    }
//    float a = (float)sum/(float)frameVec.size();
//    printf("average: %f, min: %d\n",a,min);
//
//    Viewer viewer;
//    PointVector obs_pointVector;
//    PointVector corr_pointVector;
//    Parser config(config_file);
//
//    double diff_thres = config.getValue<double>("correspondence_diff_thres");
//    double fx = config.getValue<double>("Camera.fx");
//    double fy = config.getValue<double>("Camera.fy");
//    double cx = config.getValue<double>("Camera.cx");
//    double cy = config.getValue<double>("Camera.cy");
//
//    Graph graph;
//    graph.setGraph(frameVec,ref_pointVec);
//
////    auto graph_frameVec = graph.copyFrames(frameVec.front().getConstTwc());
//    auto graph_pointVec = graph.copyPoints(frameVec.front().getConstTwc());
//
//    for(auto edge : graph.edges_)
//    {
//        Eigen::Affine3d Twc;
//        auto node = graph.nodes_[edge.node_id_];
//        auto point = graph.points_[edge.point_id_];
//        Eigen::Vector3d p_obs, p_world, obs;
//        obs = edge.obs_;
//
//        p_obs[0] = (obs[0]-cx)*obs[2]/fx;
//        p_obs[1] = (obs[1]-cy)*obs[2]/fy;
//        p_obs[2] = obs[2];
//        Twc.matrix() = node.pose_;
//        p_obs = Twc * p_obs;
//        p_world = point.pose_;
//
//        auto diff = p_obs - p_world;
//        if(sqrt(diff.dot(diff)) > diff_thres)
//        {
//            corr_pointVector.push_back(Point(p_world));
//            obs_pointVector.push_back(Point(p_obs));
//        }
//    }
//
//
////    Eigen::Affine3d Tcw;
////    Tcw.matrix() = frameVec[0].getConstTcw();
////    for(auto& point : ref_pointVec)
////    {
////        auto pose = point.getPoseInWorld();
////        pose = Tcw * pose;
////        point.setPoint(pose);
////    }
////
////    for(auto frame : frameVec)
////    {
////        Eigen::Affine3d Twc;
////        Twc.matrix() = frame.getConstTwc();
////        for(auto id_obs : frame.mObservations)
////        {
////            Eigen::Vector3d p_obs;
////            Eigen::Vector3d p_world;
////            auto point_id = id_obs.first;
////            auto obs = id_obs.second;
////
////            p_obs[0] = (obs[0]-cx)*obs[2]/fx;
////            p_obs[1] = (obs[1]-cy)*obs[2]/fy;
////            p_obs[2] = obs[2];
////
////            p_obs = Twc * p_obs;
//////            p_obs = Tcw * p_obs;
////
////            p_world = ref_pointVec[point_id].getPoseInWorld();
////            auto diff = p_obs - p_world;
//////            if(sqrt(diff.dot(diff)) > diff_thres)
//////            {
//////                corr_pointVector.push_back(ref_pointVec[point_id]);
//////            }
////            Point p(p_obs);
////            obs_pointVector.push_back(p);
////
////        }
////    }
//
//    viewer.setRefPoints(corr_pointVector);
//    viewer.setPoints(obs_pointVector);
//    viewer.visualize();
//}