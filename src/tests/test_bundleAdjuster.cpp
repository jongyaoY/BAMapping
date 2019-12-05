#include <stdio.h>
#include "../Viewer.h"
#include "../Reader.h"
#include "../Writer.h"

#include "../util/BundleAdjuster.h"
#include "../util/bal_problem.h"
#include "../Integrater.h"
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
//    BALProblem bal_problem(argv[1], false);//"../dataset_local/problem-49-7776-pre.txt"
//    bal_problem.generateCameras();
//    bal_problem.generateObeservations();
//    bal_problem.generatePoints();

    Viewer viewer;
    Graph graph;
    Graph subGraph;
//    Reader::readFrames(&graph,"../files/cameras","../files/observations");
//    Reader::readPoints(&graph,"../files/points");
    Reader::readITEFrames(&graph,"../dataset_local/ITE_dataset/cameras.txt",
                                 "../dataset_local/ITE_dataset/observations.txt",
                                 "../dataset_local/ITE_dataset/",
                          "../dataset_local/ITE_dataset/ITE.yaml");
    Reader::readITEPoints(&graph,"../dataset_local/ITE_dataset/points.txt");
    viewer.setRefPoints(graph.getConstPoints());
    subGraph = graph.getSubGraph(130,400);
    BundleAdjuster::solve(&subGraph);

    viewer.setFrames(subGraph.getConstFrames());
    viewer.setPoints(subGraph.getConstPoints());
//    Writer::writeToFile(graph,"../files/cameras_result","points_result");
    viewer.visualize();

//    Integrater integrater;
//    auto frames = subGraph.getConstFrames();
//    integrater.init("../dataset_local/ITE_dataset/ITE.yaml");
//
//    for(int i = 0;i<frames.size();i++)
//    {
//        printf("%i\n",i);
//        if(i>frames.size())
//            break;
//        integrater.integrateFrame(frames[i]);
//
//    }
//    integrater.generateMesh(true);

    return 0;
    //    FrameVector frames;
    //    Reader::readTUMFrames(frames,"../dataset_local/fr1desk/","fr1_desk.txt","groundtruth.txt");
    //    viewer.setFrames(frames);
}
