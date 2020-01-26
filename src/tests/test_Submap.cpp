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
    std::cout<<key_frames.size()<<std::endl;

    Graph graph;

    graph.setGraph(key_frames,ref_pointVec);

    auto subgraphs = Graph::spliteIntoSubgraphs(100,0,graph);
//    BundleAdjuster::optimize(graph, "../dataset/ITE_Long/ITE.yaml");
    for(auto& subgraph : subgraphs)
    {
    BundleAdjuster::optimize(subgraph, "../dataset/ITE_Long/ITE.yaml");
    }
    auto globalgraph = Graph::generateGlobalGraph(0,graph,subgraphs);
    BundleAdjuster::optimizeGlobal(globalgraph, "../dataset/ITE_Long/ITE.yaml");
    auto pointVec = globalgraph.copyPoints(frameVec.front().getConstTwc());
    auto frames = globalgraph.copyFrames(frameVec.front().getConstTwc());
//    auto pointVec = subgraphs[0].copyPoints(frameVec.front().getConstTwc());
//    auto frames = subgraphs[0].copyFrames(frameVec.front().getConstTwc());



    Viewer viewer;
    viewer.setFrames(frames);
    viewer.setPoints(pointVec);
    viewer.setRefPoints(ref_pointVec);
    viewer.visualize();

    Integrater::integrateGraph(subgraphs[0],config_file,mesh_file,key_frames.front().getConstTwc());
}