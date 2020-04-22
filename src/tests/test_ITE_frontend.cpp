//
// Created by jojo on 04.04.20.
//

#include "../io/Reader.h"
#include "../io/Writer.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "../Integrater.h"
#include "../frontend/System.h"

int main(int argc, char** argv)
{
    using namespace BAMapping;
    if(argc < 2)
    {
        printf("usage: ../path_to_data_set/ \n");
        return 0;
    }
    std::string dataset_path = argv[1];
    std::string voc_path = dataset_path + "ORBvoc.yml.gz";
    std::string obs_path = dataset_path + "observations.txt";
    std::string point_path = dataset_path + "points.txt";
    std::string cam_file = dataset_path + "cameras.txt";
    std::string out_cam_file = dataset_path + "cameras_result.txt";
    std::string config_file = dataset_path + "ITE_frontend.yaml";

    auto frameVec = io::Reader::readITEFrames(cam_file.c_str(),
                                              "",
                                              dataset_path.c_str(),1);
    FrontEnd::System frontend;
    frontend.runWithInitialGuess(frameVec,config_file);
    Writer::writeObservations(frameVec,obs_path.c_str());
    Writer::writePoints(frontend.mMap.getMapPoints(),point_path.c_str());
    Writer::writePoses(frameVec,out_cam_file.c_str());

    Viewer viewer;
    viewer.setFrames(frameVec);
    viewer.setPoints(frontend.mMap.getMapPoints());
    viewer.visualize();

    Graph graph;
    graph.setGraph(frameVec,{});
//    Integrater::integrateGraph(graph,config_file.c_str(),ply_path.c_str(), false,Mat4::Identity(),true);


}