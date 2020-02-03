//
// Created by jojo on 07.12.19.
//
//#define Debug_Local_map
#include "../io/Reader.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "../BundleAdjuster.h"
#include "../Integrater.h"
#include <sys/stat.h>

int main(int argc, char** argv)
{
    using namespace BAMapping;
    std::string dataset_path = "../dataset/ITE_Long/";

    auto frameVec = io::Reader::readITEFrames((dataset_path + "cameras.txt").c_str(),
                                              (dataset_path + "observations.txt").c_str(),
                                              dataset_path.c_str(),1);
    auto ref_pointVec = io::Reader::readPoints((dataset_path + "points.txt").c_str());

    std::string config_file = dataset_path + "ITE.yaml";
    std::string mesh_file = dataset_path + "final.ply";

    auto key_frames = FrameMethods::filterFrames(config_file.c_str(),frameVec);
    std::cout<<frameVec.size()<<std::endl;
//    std::cout<<key_frames.size()<<std::endl;

    Graph graph;

    graph.setGraph(key_frames,ref_pointVec);
    Parser config(config_file);
    int n = config.getValue<int>("n_frames_per_fragment");
    auto subgraphs = Graph::spliteIntoSubgraphs(n,0,graph);
    int id = 0;
    std::string temp_path = dataset_path + "temp/";
    std::vector<std::string> plyNames;
    for(auto& subgraph : subgraphs)
    {
        std::cout<<"optimizing: "<<id<<"/"<<subgraphs.size()<<std::endl;
        std::string plyName = temp_path + "fragment" + std::to_string(id)+".ply";
        BundleAdjuster::optimize(subgraph, "../dataset/ITE_Long/ITE.yaml");
        Integrater::integrateGraph(subgraph, config_file.c_str(), plyName.c_str(),true,Mat4::Identity(),false);

        plyNames.push_back(plyName);
        id++;

    }

    auto globalgraph = Graph::generateGlobalGraph(0,graph,subgraphs);
    BundleAdjuster::optimizeGlobal(globalgraph, config_file.c_str(),plyNames);
//    auto pointVec = globalgraph.copyPoints(frameVec.front().getConstTwc());
//    auto frames = globalgraph.copyFrames(frameVec.front().getConstTwc());
//    int id = 0;
//    BundleAdjuster::optimize(subgraphs[id], "../dataset/ITE_Long/ITE.yaml");

//    auto pointVec = subgraphs[id].copyPoints(globalgraph.nodes_[id].pose_);
//    auto frames = subgraphs[id].copyFrames(globalgraph.nodes_[id].pose_);

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