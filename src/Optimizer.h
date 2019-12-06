//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_OPTIMIZER_H
#define BAMAPPING_OPTIMIZER_H

#include "ceres/ceres.h"
#include "Graph.h"
#include "Error.h"

namespace BAMapping
{
    class Optimizer
    {
    public:
        void interGraphOptimize(std::vector<Graph*> pGlobalGraphVec);
        //multi threads optimization
        static void localGraphOptimize(Graph* pLocalGraph);
        static void init(std::string configFile);

    private:
        static void builProblem(Graph* pGraph, ceres::Problem* problem,double** cam_param,double** point_param);
        static void SetMinimizerOptions(ceres::Solver::Options* options);
        static void SetLinearSolver(ceres::Solver::Options* options);
    };
}//end of namespace



#endif //BAMAPPING_OPTIMIZER_H
