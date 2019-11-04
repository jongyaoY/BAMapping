#ifndef BUNDLEADJUSTER_H
#define BUNDLEADJUSTER_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>


#include "ceres/ceres.h"
#include "ceres/rotation.h"
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

    static double* parameters;

};


struct Error
{
   Error(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const
  {
      T p[3];
      ceres::AngleAxisRotatePoint(camera, point, p);
//      QuaternionRotatePoint(camera, point, p);

      // camera[4,5,6] are the translation.
      p[0] += camera[3];
      p[1] += camera[4];
      p[2] += camera[5];

      // Compute the center of distortion. The sign change comes from
      // the camera model that Noah Snavely's Bundler assumes, whereby
      // the camera coordinate system has a negative z axis.
      const T xp = - p[0] / p[2];
      const T yp = - p[1] / p[2];

      // Apply second and fourth order radial distortion.
      const T& l1 = camera[7];
      const T& l2 = camera[8];
      const T r2 = xp*xp + yp*yp;
      const T distortion = 1.0 + r2  * (l1 + l2  * r2);


      // Compute final projected point position.
      const T& focal = camera[6];
      const T predicted_x = focal * distortion * xp;
      const T predicted_y = focal * distortion * yp;

      // The error is the difference between the predicted and observed position.
      residuals[0] = predicted_x - observed_x;
      residuals[1] = predicted_y - observed_y;
    return true;
  }

   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y) {
     return (new ceres::AutoDiffCostFunction<Error, 2, 9, 3>(
                 new Error(observed_x, observed_y)));
   }

  double observed_x;
  double observed_y;
};


#endif // BUNDLEADJUSTER_H
