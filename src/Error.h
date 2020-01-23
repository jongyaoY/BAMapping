//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_ERROR_H
#define BAMAPPING_ERROR_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
struct ReprojectionError_2D {
    ReprojectionError_2D(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        const T xp =  p[0] / p[2];
        const T yp =  p[1] / p[2];


        // Compute final projected point position.
        const T& focal = camera[6];
        const T predicted_x = fx_ * xp + cx_;
        const T predicted_y = fy_ * yp + cy_;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError_2D, 2, 6, 3>(
                new ReprojectionError_2D(observed_x, observed_y)));
    }
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

    double observed_x;
    double observed_y;
};



struct AlignmentError_3D
{
    AlignmentError_3D(double observed_u, double observed_v, double observed_d)
            : u(observed_u), v(observed_v), d(observed_d) {}

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

        p_obs[0] = T((u-cx_)*d/fx_);
        p_obs[1] = T((v-cy_)*d/fy_);
        p_obs[2] = T(d);

//        T weight[3];
//        weight[0] = T(1.0/(d*d));
//        weight[1] = T(1.0/(d*d));
//        weight[2] = T(1.0/(d*d));

        residuals[0] = (p[0] - p_obs[0]);
        residuals[1] = (p[1] - p_obs[1]);
        residuals[2] = (p[2] - p_obs[2]);

        T norm = residuals[0]*residuals[0] + residuals[1]*residuals[1] + residuals[2]*residuals[2];
//        if(sqrt(norm) > T(0.05))
//        {
////            printf("norm:%lf, %lf %lf %lf\n",norm,residuals[0],residuals[1],residuals[2]);
//            residuals[0] = T(0);
//            residuals[1] = T(0);
//            residuals[2] = T(0);
//        }
        return true;
    }

    static ceres::CostFunction* Create(const double observed_u,
                                       const double observed_v,
                                       const double observed_d) {
        return (new ceres::AutoDiffCostFunction<AlignmentError_3D, 3, 6, 3>(
                new AlignmentError_3D(observed_u, observed_v,observed_d)));
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



struct ReprojectionError_3D
{
    ReprojectionError_3D(double observed_u, double observed_v, double observed_d)
            : u(observed_u), v(observed_v), d(observed_d) {}

    template <typename T>
    bool operator()(const T* const intrinsics,
                    const T* const camera,
                    const T* const point,
                    T* residuals) const
    {
        T p[3];
        T p_obs[3];
//        double p_[3];
//        double p_obs_[3];

        ceres::AngleAxisRotatePoint(camera, point, p);


        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        const T& fx_ = intrinsics[0];
        const T& fy_ = intrinsics[1];
        const T& cx_ = intrinsics[2];
        const T& cy_ = intrinsics[3];

        const T& u_ = T (u);
        const T& v_ = T (v);
        const T& d_ = T (d);

        p_obs[0] = (u_-cx_)*d_/fx_;
        p_obs[1] = (v_-cy_)*d_/fy_;
        p_obs[2] = d_;


        residuals[0] = (p[0] - p_obs[0]);
        residuals[1] = (p[1] - p_obs[1]);
        residuals[2] = (p[2] - p_obs[2]);

        T norm = residuals[0]*residuals[0] + residuals[1]*residuals[1] + residuals[2]*residuals[2];

//        printf("%lf %lf %lf, %lf %lf %lf\n",(double)p[0]);
//        if(sqrt(norm) > T(0.08))
//        {
//            residuals[0] = T(0);
//            residuals[1] = T(0);
//            residuals[2] = T(0);
//        }
        return true;
    }

    static ceres::CostFunction* Create(const double observed_u,
                                       const double observed_v,
                                       const double observed_d) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError_3D, 3, 4, 6, 3>(
                new ReprojectionError_3D(observed_u, observed_v,observed_d)));
    }

    double u;
    double v;
    double d;
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
        p_obs[0] = T(x);
        p_obs[1] = T(y);
        p_obs[2] = T(z);


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

double ReprojectionError_2D::fx_;
double ReprojectionError_2D::fy_;
double ReprojectionError_2D::cx_;
double ReprojectionError_2D::cy_;

#endif //BAMAPPING_ERROR_H
