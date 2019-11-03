#include "stdio.h"
#include "../util/BundleAdjuster.h"
#include "../util/bal_problem.h"
#include "../Reader.h"

int main(int argc, char** argv)
{
    printf("test\n");
    BALProblem bal_problem("problem-73-11032-pre.txt", true);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1,
                        0.1,
                        0.1);
    bal_problem.generateCameras();
    bal_problem.generateObeservations();
    bal_problem.generatePoints();

    Reader reader;
    Graph graph;
    reader.readFrames(&graph,"../files/cameras","../files/observations");
    return 0;
}
