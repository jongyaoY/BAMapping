#include <stdio.h>
#include "../Viewer.h"
#include "../Reader.h"
#include "../Writer.h"

#include "../util/BundleAdjuster.h"
#include "../util/bal_problem.h"

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    BALProblem bal_problem("../dataset_local/problem-21-11315-pre.txt", false);
    bal_problem.generateCameras();
    bal_problem.generateObeservations();
    bal_problem.generatePoints();

    Viewer viewer;
    Graph graph;
    Reader::readFrames(&graph,"../files/cameras","../files/observations");
    Reader::readPoints(&graph,"../files/points");
    viewer.setRefPoints(graph.getConstPoints());
    BundleAdjuster::solve(&graph);
    viewer.setFrames(graph.getConstFrames());
    viewer.setPoints(graph.getConstPoints());
    Writer::writeToFile(graph,"../files/cameras_result","../files/points_result");
    viewer.visualize();
    return 0;
}
