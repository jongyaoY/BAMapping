#include <stdio.h>
#include "../Viewer.h"
#include "../Reader.h"
#include "../Writer.h"

#include "../util/BundleAdjuster.h"
#include "../util/bal_problem.h"

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
                                 "../dataset_local/ITE_dataset/");
    Reader::readITEPoints(&graph,"../dataset_local/ITE_dataset/points.txt");
    viewer.setRefPoints(graph.getConstPoints());
    subGraph = graph.getSubGraph(1000,1400);
    BundleAdjuster::solve(&subGraph);
//    BundleAdjuster::solve(&graph);
    viewer.setFrames(subGraph.getConstFrames());
    viewer.setPoints(subGraph.getConstPoints());
//    Writer::writeToFile(graph,"../files/cameras_result","points_result");
    viewer.visualize();
    return 0;
    //    FrameVector frames;
    //    Reader::readTUMFrames(frames,"../dataset_local/fr1desk/","fr1_desk.txt","groundtruth.txt");
    //    viewer.setFrames(frames);
}
