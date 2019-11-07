#include <stdio.h>
#include "../Viewer.h"
#include "../Reader.h"

#include "../util/BundleAdjuster.h"
#include "../util/bal_problem.h"

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    BALProblem bal_problem("../files/problem-21-11315-pre.txt", false);
    bal_problem.generateCameras();
    bal_problem.generateObeservations();
    bal_problem.generatePoints();

    Reader reader;
    Viewer viewer;
    Graph graph;
    reader.readFrames(&graph,"../files/cameras","../files/observations");
    reader.readPoints(&graph,"../files/points");
    BundleAdjuster BA;
    viewer.setRefPoints(graph.getConstPoints());
    BA.solve(&graph);
    viewer.setFrames(graph.getConstFrames());
    viewer.setPoints(graph.getConstPoints());
    viewer.visualize();
    return 0;
}
