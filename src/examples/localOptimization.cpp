//
// Created by jojo on 06.03.20.
//
#include "io/Reader.h"
#include "Viewer.h"
#include "Graph.h"
#include "BundleAdjuster.h"
#include "Integrater.h"

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
    std::string config_file = dataset_path + "ITE.yaml";
    std::string temp_path = dataset_path + "temp/";
    std::string result_graph_file = dataset_path + "result_graph.json";


    auto frameVec = io::Reader::readITEFrames(cam_path.c_str(),
                                              (dataset_path + "observations.txt").c_str(),
                                              dataset_path.c_str(),1);
    auto ref_pointVec = io::Reader::readPoints((dataset_path + "points.txt").c_str());


    Parser config(config_file);
    int n = config.getValue<int>("n_frames_per_fragment");
    bool dense = config.getValue<bool>("global_use_dense_term");

    Graph graph;
    graph.setGraph(frameVec,ref_pointVec);
//    Graph::ReadFromeFile(graph,result_graph_file.c_str());
    auto subgraphs = Graph::spliteIntoSubgraphs(n,graph);
    for(int id = 0; id < subgraphs.size(); id++)
    {
        std::string graph_file = temp_path + "fragment" + std::to_string(id)+".json";
        std::string plyName = temp_path + "fragment" + std::to_string(id)+".ply";
        Graph& subgraph = subgraphs[id];

        BundleAdjuster::optimize(subgraph, config_file.c_str(),false);
        Graph::WriteToFile(subgraph,graph_file.c_str());
        if(dense)
            Integrater::integrateGraph(subgraph, config_file.c_str(), plyName.c_str(),true,Mat4::Identity(),false);
    }

}
