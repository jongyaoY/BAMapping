//
// Created by jojo on 14.12.19.
//
#include "../io/Reader.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "../Optimizer.h"
#include "../Integrater.h"

int main(int argc,char** argv)
{
    BAMapping::Graph graph;
    BAMapping::io::Reader::readTUMData(&graph,"camera.txt",
            "../dataset_local/fr2_large_no_loop/obs.txt",
            "../dataset_local/fr2_large_no_loop/points.txt",
            "../dataset_local/fr2_large_no_loop/",
            "ass.txt",
            "../dataset_local/fr2_large_no_loop/TUM2.yaml");
    BAMapping::Viewer viewer;
    BAMapping::Graph::setAsRootGraph(&graph);
//    viewer.setFrames(graph.getConstGlobalFrames());
//    viewer.setPoints(graph.getConstGlobalPoints());
//    viewer.visualize();

//    graph.splitInto(2);
    BAMapping::Optimizer::init("../dataset_local/fr1desk/TUM1.yaml");
//    BAMapping::Optimizer::localGraphOptimize(submaps[0].get());
    BAMapping::Optimizer::optimize(&graph,10);

    auto submaps = graph.getSubmaps();
//    viewer.setFrames(submaps[0]->getLocalConstFrames());
//    viewer.setPoints(submaps[0]->getLocalConstPoints());
    viewer.setFrames(graph.getConstGlobalFrames());
    viewer.setPoints(graph.getConstGlobalPoints());
    viewer.visualize();

    Integrater integrater;
    integrater.init("../dataset_local/fr2_large_no_loop/TUM2.yaml");

    int i = 0;
//    for(auto frame : submaps[1]->getLocalConstFrames())
    for(auto frame : graph.getConstGlobalFrames())

    {
        printf("%i\n",i+1);
        if(i>1000)
            break;
        integrater.integrateFrame(frame);
        i++;
    }

    integrater.generateMesh(true);
    return 0;
}
