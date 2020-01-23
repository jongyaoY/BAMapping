//
// Created by jojo on 30.11.19.
//
#include <iostream>
#include <memory>
#include <thread>


#include "../Integrater.h"
#include "../Reader.h"
int main(int argc, char** argv)
{
    Integrater integrater;
    FrameVector frames;
    Graph graph;
    //tum
    Reader::readTUMFrames(frames,"../dataset_local/fr1desk/","fr1_desk.txt","camera.txt");
    integrater.init("../dataset_local/fr1desk/TUM1.yaml");
    int i = 0;

    for(auto frame : frames)
    {
        printf("%i\n",i+1);

        integrater.integrateFrame(frame);
        if(i>200)
            break;
        else
            i++;
    }
    integrater.generateMesh(true);
//    //ITE
//    Reader::readITEFrames(&graph,
//            "../dataset_local/ITE_dataset/cameras.txt",
//            "../dataset_local/ITE_dataset/observations.txt",
//            "../dataset_local/ITE_dataset/",
//            "../dataset_local/ITE_dataset/ITE.yaml");
//
//    frames = graph.getConstFrames();
//    integrater.init("../dataset_local/ITE_dataset/ITE.yaml");
//
//    int offset = 29;
//    int begin = 170 - offset;
//    int end = 300 - offset;
//    for(int i = begin;i<end;i++)
//    {
//        printf("%i\n",i+offset);
//        if(i>frames.size())
//            break;
//        integrater.integrateFrame(frames[i]);
//
//    }
//    integrater.generateMesh(true);
    return 0;
}

