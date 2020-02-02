//
// Created by jojo on 07.12.19.
//
//#define Debug_Local_map
#include "../io/Reader.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "../BundleAdjuster.h"
#include "../Integrater.h"

int main(int argc, char** argv)
{
    using namespace BAMapping;
    auto frameVec = io::Reader::readITEFrames("../dataset/ITE_Long/cameras.txt",
                                              "../dataset/ITE_Long/observations.txt",
                                              "../dataset/ITE_Long/",1);
    auto ref_pointVec = io::Reader::readPoints("../dataset/ITE_Long/points.txt");


    const char* config_file = "../dataset/ITE_Long/ITE.yaml";
    const char* mesh_file = "final.ply";

    auto key_frames = FrameMethods::filterFrames(config_file,frameVec);
    std::cout<<frameVec.size()<<std::endl;
//    std::cout<<key_frames.size()<<std::endl;

    Graph graph;

    graph.setGraph(key_frames,ref_pointVec);
    Parser config(config_file);
    int n = config.getValue<int>("n_frames_per_fragment");
    auto subgraphs = Graph::spliteIntoSubgraphs(n,0,graph);
    auto globalgraph = Graph::generateGlobalGraph(0,graph,subgraphs);
    int id = 0;
    for(auto& subgraph : subgraphs)
    {
        BundleAdjuster::optimize(subgraph, "../dataset/ITE_Long/ITE.yaml");
//        Integrater::integrateGraph(subgraph,config_file,mesh_file,frameVec.front().getConstTwc());
        id++;

    }

    BundleAdjuster::optimizeGlobal(globalgraph, "../dataset/ITE_Long/ITE.yaml");
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

    Integrater::integrateGraph(resultGraph,config_file,mesh_file,frameVec.front().getConstTwc());
}