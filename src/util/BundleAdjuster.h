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






















DEFINE_string(input, "", "Input File name");
DEFINE_string(trust_region_strategy, "levenberg_marquardt",
              "Options are: levenberg_marquardt, dogleg.");
DEFINE_string(dogleg, "traditional_dogleg", "Options are: traditional_dogleg,"
              "subspace_dogleg.");

DEFINE_bool(inner_iterations, false, "Use inner iterations to non-linearly "
            "refine each successful trust region step.");

DEFINE_string(blocks_for_inner_iterations, "automatic", "Options are: "
            "automatic, cameras, points, cameras,points, points,cameras");

DEFINE_string(linear_solver, "sparse_schur", "Options are: "
              "sparse_schur, dense_schur, iterative_schur, sparse_normal_cholesky, "
              "dense_qr, dense_normal_cholesky and cgnr.");
DEFINE_bool(explicit_schur_complement, false, "If using ITERATIVE_SCHUR "
            "then explicitly compute the Schur complement.");
DEFINE_string(preconditioner, "jacobi", "Options are: "
              "identity, jacobi, schur_jacobi, cluster_jacobi, "
              "cluster_tridiagonal.");
DEFINE_string(visibility_clustering, "canonical_views",
              "single_linkage, canonical_views");

DEFINE_string(sparse_linear_algebra_library, "suite_sparse",
              "Options are: suite_sparse and cx_sparse.");
DEFINE_string(dense_linear_algebra_library, "eigen",
              "Options are: eigen and lapack.");
DEFINE_string(ordering, "automatic", "Options are: automatic, user.");

DEFINE_bool(use_quaternions, false, "If true, uses quaternions to represent "
            "rotations. If false, angle axis is used.");
DEFINE_bool(use_local_parameterization, false, "For quaternions, use a local "
            "parameterization.");
DEFINE_bool(robustify, false, "Use a robust loss function.");

DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the "
             "accuracy of each linear solve of the truncated newton step. "
             "Changing this parameter can affect solve performance.");

DEFINE_int32(num_threads, 1, "Number of threads.");
DEFINE_int32(num_iterations, 5, "Number of iterations.");
DEFINE_double(max_solver_time, 1e32, "Maximum solve time in seconds.");
DEFINE_bool(nonmonotonic_steps, false, "Trust region algorithm can use"
            " nonmonotic steps.");

DEFINE_double(rotation_sigma, 0.0, "Standard deviation of camera rotation "
              "perturbation.");
DEFINE_double(translation_sigma, 0.0, "Standard deviation of the camera "
              "translation perturbation.");
DEFINE_double(point_sigma, 0.0, "Standard deviation of the point "
              "perturbation.");
DEFINE_int32(random_seed, 38401, "Random seed used to set the state "
             "of the pseudo random number generator used to generate "
             "the pertubations.");
DEFINE_bool(line_search, false, "Use a line search instead of trust region "
            "algorithm.");
DEFINE_string(initial_ply, "", "Export the BAL file data as a PLY file.");
DEFINE_string(final_ply, "", "Export the refined BAL file data as a PLY "
              "file.");
#endif // BUNDLEADJUSTER_H
