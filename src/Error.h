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
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        // Compute final projected point position.

        T predicted_u = fx_ * xp + cx_;
        T predicted_v = fy_ * yp + cy_;
        T predicted_d = p[2];

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_u - u;
        residuals[1] = predicted_v - v;
        if(d>0)
            residuals[2] = predicted_d - d;
        else
            residuals[2] = (T) 0;
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

double AlignmentError_3D::fx_;
double AlignmentError_3D::fy_;
double AlignmentError_3D::cx_;
double AlignmentError_3D::cy_;


#endif //BAMAPPING_ERROR_H
