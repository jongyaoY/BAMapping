//
// Created by jojo on 07.12.19.
//
//#define Debug_Local_map
#include "../io/Reader.h"
#include "../Graph.h"
#include "../Viewer.h"
#include "../BundleAdjuster.h"
#include "../Integrater.h"

int main(int argc, char** argv)
{
    using namespace BAMapping;
    auto frameVec = io::Reader::readITEFrames("../dataset/ITE_Long/cameras.txt",
                                              "../dataset/ITE_Long/observations.txt",
                                              "../dataset/ITE_Long/",1);
    auto pointVec = io::Reader::readPoints("../dataset/ITE_Long/points.txt");

    const char* config_file = "../dataset/ITE_Long/ITE.yaml";
    const char* mesh_file = "final.ply";
//    auto refPoint = pointVec;
//    frameVec.erase(frameVec.begin(),frameVec.begin() + 200);
//    frameVec.erase(frameVec.begin()+200, frameVec.end());

    Graph graph;

    graph.setGraph(frameVec,pointVec);

    BundleAdjuster::optimize(graph, "../dataset/ITE_Long/ITE.yaml");
    auto refPoint = graph.copyPoints(frameVec.front().getConstTwc());
    auto frames = graph.copyFrames(frameVec.front().getConstTwc());
//
//
//    Viewer viewer;
//    viewer.setFrames(frames);
//    viewer.setPoints(pointVec);
//    viewer.setRefPoints(refPoint);
//    viewer.visualize();

    Integrater::integrateGraph(graph,config_file,mesh_file,frameVec.front().getConstTwc());
}