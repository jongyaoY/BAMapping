//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_ERROR_H
#define BAMAPPING_ERROR_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
struct AlignmentError_3D
{
    AlignmentError_3D(double observed_x, double observed_y, double observed_z)
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


        residuals[0] = p[0] - p_obs[0];
        residuals[1] = p[1] - p_obs[1];
        residuals[2] = p[2] - p_obs[2];
        return true;
    }

    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y,
                                       const double observed_z) {
        return (new ceres::AutoDiffCostFunction<AlignmentError_3D, 3, 6, 3>(
                new AlignmentError_3D(observed_x, observed_y,observed_z)));
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

struct AlignmentError_3D_Direct
{
    AlignmentError_3D_Direct(double observed_x, double observed_y, double observed_z)
            : x(observed_x), y(observed_y), z(observed_z) {}

    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const
    {
        T p[3];
        T p_obs[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3,4,5] are the translation.
        //predicted point
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        //observed point
        p_obs[0] = (T) x;
        p_obs[1] = (T) y;
        p_obs[2] = (T) z;


        residuals[0] = p[0] - p_obs[0];
        residuals[1] = p[1] - p_obs[1];
        residuals[2] = p[2] - p_obs[2];
        return true;
    }

    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y,
                                       const double observed_z) {
        return (new ceres::AutoDiffCostFunction<AlignmentError_3D_Direct, 3, 6, 3>(
                new AlignmentError_3D_Direct(observed_x, observed_y,observed_z)));
    }

    double x;
    double y;
    double z;
};

double AlignmentError_3D::fx_;
double AlignmentError_3D::fy_;
double AlignmentError_3D::cx_;
double AlignmentError_3D::cy_;


#endif //BAMAPPING_ERROR_H
