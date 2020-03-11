//
// Created by jojo on 07.12.19.
//
//#define Debug_Local_map
#include "../io/Reader.h"
#include "../io/Writer.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "../BundleAdjuster.h"
#include "../Integrater.h"

int main(int argc, char** argv)
{
    using namespace BAMapping;
    std::string dataset_path = "../dataset/mpu/";//"../dataset/ITE_Office/";
    std::string cam_path = dataset_path + argv[1];
    auto frameVec = io::Reader::readITEFrames(cam_path.c_str(),
                                              (dataset_path + "observations.txt").c_str(),
                                              dataset_path.c_str(),1);
    auto ref_pointVec = io::Reader::readPoints((dataset_path + "points.txt").c_str());

    std::string config_file = dataset_path + "ITE.yaml";
    std::string mesh_file = dataset_path + "final.ply";
    std::string output_cam_file = dataset_path + "cameras_result.txt";
    std::string output_point_file = dataset_path + "points_result.txt";
    std::string output_image_path_file = dataset_path + "image_paths.txt";
    std::string init_cam_file_tum_format = dataset_path + "init_tum_format.txt";
    std::string result_cam_file_tum_format = dataset_path + "result_tum_format.txt";

//    FrameMethods::filterObservations(frameVec,ref_pointVec,config_file.c_str());
//    auto key_frames = FrameMethods::filterFrames(config_file.c_str(),frameVec);
    std::cout<<frameVec.size()<<std::endl;
//    std::cout<<key_frames.size()<<std::endl;

    Graph graph;
    graph.setGraph(frameVec,ref_pointVec);
    Parser config(config_file);


    bool use_submap = config.getValue<bool>("use_submap");

    if(use_submap)
    {
        int n = config.getValue<int>("n_frames_per_fragment");
        auto subgraphs = Graph::spliteIntoSubgraphs(n,0,graph);
        int id = 0;
        std::string temp_path = dataset_path + "temp/";
        std::vector<std::string> plyNames;
        id = std::stoi(argv[2]);

        for(auto& subgraph : subgraphs)
        {
            subgraph = subgraphs[id];
            std::cout<<"optimizing: "<<id<<"/"<<subgraphs.size()<<std::endl;
            std::string plyName = temp_path + "fragment" + std::to_string(id)+".ply";
            BundleAdjuster::optimize(subgraph, config_file.c_str(),false);
            Integrater::integrateGraph(subgraph, config_file.c_str(), plyName.c_str(),true,Mat4::Identity(),true);

            plyNames.push_back(plyName);
            auto pointVec_ = subgraph.copyPoints(frameVec.front().getConstTwc());
            auto frames_ = subgraph.copyFrames(frameVec.front().getConstTwc());
            id++;
//            Viewer viewer_;
//            viewer_.setFrames(frames_);
//            viewer_.setPoints(pointVec_);
//            viewer_.setRefPoints(ref_pointVec);
//            viewer_.visualize();
        }

        auto globalgraph = Graph::generateGlobalGraph(0,graph,subgraphs);
        BundleAdjuster::optimizeGlobal(globalgraph, config_file.c_str(),plyNames);

        Graph::markSeperators(globalgraph,subgraphs);
        id = 0;
        for(auto& subgraph : subgraphs)
        {
            std::cout<<"optimizing with fixed seperators: "<<id<<"/"<<subgraphs.size()<<std::endl;
            BundleAdjuster::optimize(subgraph, config_file.c_str(),true);
            id++;
        }

        auto resultGraph = Graph::generateResultGraph(0,globalgraph,subgraphs);


        auto pointVec = resultGraph.copyPoints(frameVec.front().getConstTwc());
        auto frames = resultGraph.copyFrames(frameVec.front().getConstTwc());
        Viewer viewer;
        viewer.setFrames(frames);
        viewer.setPoints(pointVec);
        viewer.setRefPoints(ref_pointVec);
        viewer.visualize();

        Integrater::integrateGraph(resultGraph,config_file.c_str(),mesh_file.c_str(),false,frameVec.front().getConstTwc(),true);

    }
    else
    {

//        Writer::writeToFileTUMFormat(graph,init_cam_file_tum_format.c_str());
        std::cout<<"read point: "<<ref_pointVec.size()<<std::endl;
        std::cout<<"point in graph: "<<graph.points_.size()<<std::endl;
//        BundleAdjuster::optimize(graph, config_file.c_str(),false);
//        Writer::writeToFile(graph, output_cam_file.c_str(),output_point_file.c_str(),output_image_path_file.c_str());
//        Writer::writeToFileTUMFormat(graph,result_cam_file_tum_format.c_str());

        auto pointVec = graph.copyPoints(frameVec.front().getConstTwc());
        auto frames = graph.copyFrames(frameVec.front().getConstTwc());
        Viewer viewer;
        viewer.setFrames(frames);
        viewer.setPoints(pointVec);
        viewer.setRefPoints(ref_pointVec);
        viewer.visualize();

        Integrater::integrateGraph(graph,config_file.c_str(),mesh_file.c_str(),false,frameVec.front().getConstTwc(),true);
    }




}


//    auto pointVec = graph.copyPoints(frameVec.front().getConstTwc());
//    auto frames = graph.copyFrames(frameVec.front().getConstTwc());
//    Viewer viewer;
//    viewer.setFrames(frames);
//    viewer.setPoints(pointVec);
//    viewer.setRefPoints(ref_pointVec);
//    viewer.visualize();

//    auto graph_ref_pointVec = graph.copyPoints(Mat4::Identity());
//    int n = config.getValue<int>("n_frames_per_fragment");
//    auto subgraphs = Graph::spliteIntoSubgraphs(n,0,graph);
//    auto globalgraph = Graph::generateGlobalGraph(0,graph,subgraphs);

//    for(int i = 0; i < subgraphs.size(); i++)
//    {
////        int i = 0;
//        auto subgraph = subgraphs[i];
//        BundleAdjuster::optimize(subgraph, config_file.c_str(),false);
//
//        auto pointVec = subgraph.copyPoints(globalgraph.nodes_[i].pose_);
//        auto frames = subgraph.copyFrames(globalgraph.nodes_[i].pose_);
//
////        double fx = config.getValue<double>("Camera.fx");
////        double fy = config.getValue<double>("Camera.fy");
////        double cx = config.getValue<double>("Camera.cx");
////        double cy = config.getValue<double>("Camera.cy");
////        PointVector obs_point_vec;
////        for(auto edge : subgraph.edges_)
////        {
////            auto node = subgraph.nodes_[edge.node_id_];
////            auto obs = edge.obs_;
////            Eigen::Affine3d Twc;
////
////            Eigen::Vector3d p_obs;
////            p_obs[0] = (obs[0]-cx)*obs[2]/fx;
////            p_obs[1] = (obs[1]-cy)*obs[2]/fy;
////            p_obs[2] = obs[2];
////            Twc.matrix() = node.pose_;
////            p_obs = Twc * p_obs;
////            Twc.matrix() = globalgraph.nodes_[i].pose_;
////            p_obs = Twc * p_obs;
////            obs_point_vec.push_back(Point(p_obs));
////        }
//
//
//        Viewer viewer;
//        viewer.setRefPoints(graph_ref_pointVec);
//        viewer.setFrames(frames);
//        viewer.setPoints(pointVec);
//        viewer.visualize();
//
//        Integrater::integrateGraph(graph,config_file.c_str(),mesh_file.c_str(),false,graph.nodes_[0].pose_,true);
//    }