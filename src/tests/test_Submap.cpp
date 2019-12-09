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
    graph.splitInto(30);
    auto submaps = graph.getSubmaps();


    BAMapping::Optimizer::init("../dataset_local/ITE_dataset/ITE.yaml");
//    BAMapping::Optimizer::localGraphOptimize(submaps[0].get());
//    BAMapping::Optimizer::localGraphOptimize(submaps[1].get());

//    viewer.setFrames(submaps[10]->getLocalConstFrames());
//    viewer.setPoints(submaps[10]->getLocalConstPoints());

//    viewer.setFrames(graph.getConstGlobalFrames());
//    viewer.setPoints(graph.getConstGlobalPoints());
//    viewer.visualize();
    Integrater integrater;
    integrater.init("../dataset_local/ITE_dataset/ITE.yaml");

    int i = 0;
    for(auto frame : submaps[0]->getLocalConstFrames())
    {
        printf("%i\n",i+1);

        integrater.integrateFrame(frame);
        if(i>200)
            ;
//            break;
        else
            i++;
    }
//    for(auto frame : submaps[1]->getLocalConstFrames())
//    {
//        printf("%i\n",i+1);
//
//        integrater.integrateFrame(frame);
//        if(i>200)
//            ;
////            break;
//        else
//            i++;
//    }
    integrater.generateMesh(true);
}