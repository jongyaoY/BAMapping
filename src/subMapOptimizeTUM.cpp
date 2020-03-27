//
// Created by jojo on 26.03.20.
//
#include "io/Reader.h"
#include "Viewer.h"
#include "Graph.h"
#include "BundleAdjuster.h"
#include "Integrater.h"
#include "io/Writer.h"

#include <iostream>

int main(int argc, char** argv)
{
    using namespace BAMapping;
    if(argc < 2)
    {
        printf("usage: ../path_to_data_set/ \n");
        return 0;
    }
    std::string dataset_path = argv[1];
    std::string cam_path = dataset_path + "cameras.txt";
    std::string obs_file = dataset_path + "observations.txt";
    std::string config_file = dataset_path + "TUM.yaml";
    std::string asso_path = "ass.txt";
    std::string temp_path = dataset_path + "temp/";
    std::string result_graph_file = dataset_path + "result_graph.json";
    std::string result_cam_file_tum_format = dataset_path + "result_tum_format.txt";
    std::string mesh_file = dataset_path + "final.ply";

    auto frameVec = io::Reader::readTUMFrames(dataset_path,cam_path,asso_path);
    auto ref_pointVec = io::Reader::readPoints((dataset_path + "points.txt").c_str());
    io::Reader::readObservations(obs_file,frameVec);

    Parser config(config_file);
    int n = config.getValue<int>("n_frames_per_fragment");

    Graph graph;
    graph.setGraph(frameVec,ref_pointVec);
    std::vector<std::string> plyNames;

    auto subgraphs = Graph::spliteIntoSubgraphs(n,graph);
    for(int id = 0; id < subgraphs.size(); id++)
    {
        std::string graph_file = temp_path + "fragment" + std::to_string(id)+".json";
        std::string plyName = temp_path + "fragment" + std::to_string(id)+".ply";
        Graph& subgraph = subgraphs[id];

        plyNames.push_back(plyName);
        BundleAdjuster::optimize(subgraph, config_file.c_str(),false);
        Graph::WriteToFile(subgraph,graph_file.c_str());
        Integrater::integrateGraph(subgraph, config_file.c_str(), plyName.c_str(),true,Mat4::Identity(),false);
    }
    auto globalgraph = Graph::generateGlobalGraph(graph,subgraphs);

    BundleAdjuster::optimizeGlobal(globalgraph, config_file.c_str(), plyNames);

    Graph::markSeperators(globalgraph,subgraphs);

    int id = 0;
    for(auto& subgraph : subgraphs)
    {
        std::string graph_file = temp_path + "opt_fragment" + std::to_string(id)+".json";
        std::string plyName = temp_path + "opt_fragment" + std::to_string(id)+".ply";

        std::cout<<"optimizing with fixed seperators: "<<id<<"/"<<subgraphs.size()<<std::endl;
        BundleAdjuster::optimize(subgraph, config_file.c_str(),true);
        Graph::WriteToFile(subgraph,graph_file.c_str());
//        Integrater::integrateGraph(subgraph, config_file.c_str(), plyName.c_str(),true,Mat4::Identity(),false);
        id++;
    }

    auto resultGraph = Graph::generateResultGraph(0,globalgraph,subgraphs);
    auto pointVec = resultGraph.copyPoints(frameVec.front().getConstTwc());
    auto frames = resultGraph.copyFrames(frameVec.front().getConstTwc());

    Graph::WriteToFile(resultGraph,result_graph_file.c_str());
    Writer::writeToFileTUMFormat(resultGraph,result_cam_file_tum_format.c_str());

    Viewer viewer;
    viewer.setFrames(frames);
    viewer.setPoints(pointVec);
    viewer.setRefPoints(ref_pointVec);
    viewer.visualize();

    Integrater::integrateGraph(resultGraph,config_file.c_str(),mesh_file.c_str(),false,frameVec.front().getConstTwc(),true);

    return 0;


}
