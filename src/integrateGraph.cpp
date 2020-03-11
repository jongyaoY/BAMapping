//
// Created by jojo on 09.03.20.
//
#include "Graph.h"
#include "Integrater.h"
#include <iostream>
int main(int argc, char** argv)
{
    using namespace BAMapping;
    if(argc < 4)
    {
        printf("usage: ../path_to_data_set/ graph_name mesh_name\n");
        return 0;
    }
    std::string dataset_path = argv[1];
    std::string graph_file = dataset_path + argv[2];
    std::string mesh_file = dataset_path + argv[3];
    std::string config_file = dataset_path + "ITE.yaml";

    Graph graph;
    Graph::ReadFromeFile(graph,graph_file.c_str());
    Integrater::integrateGraph(graph,config_file.c_str(),mesh_file.c_str(),false,Mat4::Identity(),true);

    return 0;
}
