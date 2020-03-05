//
// Created by jojo on 05.03.20.
//
#include "../io/Reader.h"
#include "../Viewer.h"
#include "../Graph.h"
#include "../rapidjson/document.h"
#include "../rapidjson/writer.h"
#include "../rapidjson/stringbuffer.h"
#include "../Viewer.h"
#include <iostream>
int main(int argc, char** argv)
{
    using namespace rapidjson;
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

    Graph graph;
    graph.setGraph(frameVec,ref_pointVec);

//    Graph::WriteToFile(graph,"test_graph.json");
    Graph::ReadFromeFile(graph,"test_graph.json");

    auto pointVec = graph.copyPoints(frameVec.front().getConstTwc());
    auto frames = graph.copyFrames(frameVec.front().getConstTwc());
    Viewer viewer;
    viewer.setFrames(frames);
    viewer.setPoints(pointVec);
    viewer.setRefPoints(ref_pointVec);
    viewer.visualize();

    return 0;
}
