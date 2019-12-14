//
// Created by jojo on 07.12.19.
//
#include "../io/Reader.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "../Optimizer.h"
#include "../Integrater.h"

int main(int argc, char** argv)
{

    BAMapping::Graph graph;
    BAMapping::io::Reader::readITEData(&graph,"../dataset_local/ITE_dataset/cameras.txt",
                            "../dataset_local/ITE_dataset/observations.txt",
                            "../dataset_local/ITE_dataset/points.txt",
                            "../dataset_local/ITE_dataset/",
                            "../dataset_local/ITE_dataset/ITE.yaml");
    BAMapping::Graph::setAsRootGraph(&graph);
    BAMapping::Viewer viewer;
//    graph.splitInto(50);

//    int section = std::stoi(argv[1]);

    BAMapping::Optimizer::init("../dataset_local/ITE_dataset/ITE.yaml");
    BAMapping::Optimizer::optimize(&graph,50);
//    BAMapping::Optimizer::localGraphOptimize(submaps[section].get());


//    viewer.setFrames(submaps[section]->getLocalConstFrames());
//    viewer.setPoints(submaps[section]->getLocalConstPoints());

    viewer.setFrames(graph.getConstGlobalFrames());
    viewer.setPoints(graph.getConstGlobalPoints());
    viewer.visualize();

    Integrater integrater;
    integrater.init("../dataset_local/ITE_dataset/ITE.yaml");
    auto submaps = graph.getSubmaps();

    int i = 0;
    for(auto frame : submaps[0]->getLocalConstFrames())
    {
        printf("%i\n",i+1);

        integrater.integrateFrame(frame);
        i++;
    }

    integrater.generateMesh(true);
}