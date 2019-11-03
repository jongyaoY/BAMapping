#ifndef BUNDLEADJUSTER_H
#define BUNDLEADJUSTER_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>


#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "bal_problem.h"
#include "Graph.h"

using namespace ceres;
class BundleAdjuster
{

public:
    BundleAdjuster();
    static void SetLinearSolver(Solver::Options* options);
    static void SetMinimizerOptions(Solver::Options* options);

    static void SetSolverOptions(Graph* graph, Solver::Options* options);
    static void SetOrdering(Graph* graph, Solver::Options* options);
    static void BuildProblem(Graph *graph,Problem* problem);
    static void solve(Graph *graph);

};


struct Error {
   Error(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const
  {

    return true;
  }

   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y) {
     return (new ceres::AutoDiffCostFunction<Error, 2, 10, 3>(
                 new Error(observed_x, observed_y)));
   }

  double observed_x;
  double observed_y;
};


#endif // BUNDLEADJUSTER_H
