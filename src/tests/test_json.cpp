//
// Created by jojo on 05.03.20.
//
#include "../io/Reader.h"
#include "../Viewer.h"
#include "../Graph.h"
#include "../BundleAdjuster.h"
#include "../Integrater.h"

#include <iostream>
int main(int argc, char** argv)
{
    using namespace BAMapping;
    std::string dataset_path = "../dataset/ITE_Office/";
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

    Parser config(config_file);
    Graph graph;
    graph.setGraph(frameVec,ref_pointVec);


    int n = config.getValue<int>("n_frames_per_fragment");
    std::string temp_path = dataset_path + "temp/";
    std::vector<std::string> plyNames;

    auto subgraphs = Graph::spliteIntoSubgraphs(n,graph);
    std::vector<Graph> subgraph_read;
    for(int id = 0; id < subgraphs.size(); id++)
    {
        std::string graph_file = temp_path + "fragment" + std::to_string(id)+".json";
        std::string plyName = temp_path + "fragment" + std::to_string(id)+".ply";
        plyNames.push_back(plyName);
        Graph subgraph;
        Graph::ReadFromeFile(subgraph,graph_file.c_str());
        subgraph_read.push_back(subgraph);
    }

    auto globalgraph = Graph::generateGlobalGraph(graph,subgraph_read);
    BundleAdjuster::optimizeGlobal(globalgraph, config_file.c_str(),plyNames);

    Graph::markSeperators(globalgraph,subgraph_read);

    int id = 0;
    for(auto& subgraph : subgraph_read)
    {
        std::cout<<"optimizing with fixed seperators: "<<id<<"/"<<subgraphs.size()<<std::endl;
        BundleAdjuster::optimize(subgraph, config_file.c_str(),true);
        id++;
    }

    auto resultGraph = Graph::generateResultGraph(0,globalgraph,subgraph_read);
    auto pointVec = resultGraph.copyPoints(frameVec.front().getConstTwc());
    auto frames = resultGraph.copyFrames(frameVec.front().getConstTwc());
    Viewer viewer;
    viewer.setFrames(frames);
    viewer.setPoints(pointVec);
    viewer.setRefPoints(ref_pointVec);
    viewer.visualize();

    Integrater::integrateGraph(resultGraph,config_file.c_str(),mesh_file.c_str(),false,frameVec.front().getConstTwc(),true);

    //    for(int id = 0; id < subgraphs.size(); id++)
//    {
//        Graph& subgraph = subgraphs[id];
//        std::string plyName = temp_path + "fragment" + std::to_string(id)+".ply";
//        std::string graph_file = temp_path + "fragment" + std::to_string(id)+".json";
//
//        BundleAdjuster::optimize(subgraph, config_file.c_str(),false);
//        Graph::WriteToFile(subgraph,graph_file.c_str());
//        Integrater::integrateGraph(subgraph, config_file.c_str(), plyName.c_str(),true,Mat4::Identity(),false);
//
//    }


    return 0;
}
