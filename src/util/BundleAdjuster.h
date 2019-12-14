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

    static double** cam_param;
    static double** point_param;

};

struct Error_3D
{
   Error_3D(double observed_x, double observed_y, double observed_z)
      : u(observed_x), v(observed_y), d(observed_z) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const
  {
      T p[3];
      T p_obs[3];
      ceres::AngleAxisRotatePoint(camera, point, p);

      // camera[3,4,5] are the translation.
      p[0] += camera[3];
      p[1] += camera[4];
      p[2] += camera[5];

      p_obs[0] = (T) (u-cx_)*d/fx_;
      p_obs[1] = (T) (v-cy_)*d/fy_;
      p_obs[2] = (T) d;

      T xp = p[0] / p[2];
      T yp = p[1] / p[2];

      // Compute final projected point position.

      T predicted_u = fx_ * xp + cx_;
      T predicted_v = fy_ * yp + cy_;
      T predicted_d = p[2];

      residuals[0] = p[0] - p_obs[0];
      residuals[1] = p[1] - p_obs[1];
      residuals[2] = p[2] - p_obs[2];

      // The error is the difference between the predicted and observed position.
//      residuals[0] = predicted_u - u;
//      residuals[1] = predicted_v - v;
//      if(d>0)
//          residuals[2] = predicted_d - d;
//      else
//          residuals[2] = (T) 0;
    return true;
  }

   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y,
                                      const double observed_z) {
     return (new ceres::AutoDiffCostFunction<Error_3D, 3, 6, 3>(
                 new Error_3D(observed_x, observed_y,observed_z)));
   }

  double u;
  double v;
  double d;

  static void setIntrinsics(double fx,double fy,double cx,double cy)
  {
      fx_ = fx;
      fy_ = fy;
      cx_ = cx;
      cy_ = cy;
  }
  static double fx_;
  static double fy_;
  static double cx_;
  static double cy_;
};

double Error_3D::fx_;
double Error_3D::fy_;
double Error_3D::cx_;
double Error_3D::cy_;

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

      // camera[3,4,5] are the translation.
      p[0] += camera[3];
      p[1] += camera[4];
      p[2] += camera[5];

      // Compute the center of distortion. The sign change comes from
      // the camera model that Noah Snavely's Bundler assumes, whereby
      // the camera coordinate system has a negative z axis.
      T xp = - p[0] / p[2];
      T yp = - p[1] / p[2];

      // Apply second and fourth order radial distortion.
      const T& l1 = camera[7];
      const T& l2 = camera[8];
      T r2 = xp*xp + yp*yp;
      T distortion = 1.0 + r2  * (l1 + l2  * r2);

      // Compute final projected point position.
      const T& focal = camera[6];
      T predicted_x = focal * distortion * xp;
      T predicted_y = focal * distortion * yp;

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
