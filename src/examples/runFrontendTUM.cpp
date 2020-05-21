//
// Created by jojo on 24.03.20.
//
#include "../io/Reader.h"
//#include "../Frontend.h"
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
    std::string asso_path = "ass.txt";
    std::string config_file = dataset_path + "TUM_frontend.yaml";

    std::string cam_file = dataset_path + "cameras.txt";
    std::string point_file = dataset_path + "points.txt";
    std::string obs_file = dataset_path + "observations.txt";
    std::string ply_path = dataset_path + "result.ply";
    auto frameVec = io::Reader::readTUMFrames(dataset_path,asso_path);
    std::cout<<frameVec.size()<<std::endl;
    FrontEnd::System frontend;
    frontend.run(frameVec,config_file);

    //for non ITE dataset
    Writer::writeObservationsNormal(frameVec,obs_file.c_str());
    Writer::writePoints(frontend.mMap.getMapPoints(),point_file.c_str());
    Graph graph;
    graph.setGraph(frameVec,{});

    Writer::writeToFileTUMFormat(graph,cam_file.c_str());


//    Integrater::integrateGraph(graph,config_file.c_str(),ply_path.c_str(), false,Mat4::Identity(),true);
    Viewer viewer;
    viewer.setFrames(frameVec);
    viewer.setPoints(frontend.mMap.getMapPoints());
    viewer.visualize();

}
