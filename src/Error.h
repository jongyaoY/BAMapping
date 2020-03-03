//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_ERROR_H
#define BAMAPPING_ERROR_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
//struct ReprojectionError_2D {
//    ReprojectionError_2D(double observed_x, double observed_y)
//            : observed_x(observed_x), observed_y(observed_y) {}
//
//    template <typename T>
//    bool operator()(const T* const camera,
//                    const T* const point,
//                    T* residuals) const {
//        // camera[0,1,2] are the angle-axis rotation.
//        T p[3];
//        ceres::AngleAxisRotatePoint(camera, point, p);
//
//        // camera[3,4,5] are the translation.
//        p[0] += camera[3];
//        p[1] += camera[4];
//        p[2] += camera[5];
//
//        const T xp =  p[0] / p[2];
//        const T yp =  p[1] / p[2];
//
//
//        // Compute final projected point position.
//        const T& focal = camera[6];
//        const T predicted_x = fx_ * xp + cx_;
//        const T predicted_y = fy_ * yp + cy_;
//
//        // The error is the difference between the predicted and observed position.
//        residuals[0] = predicted_x - observed_x;
//        residuals[1] = predicted_y - observed_y;
//
//        return true;
//    }
//
//    // Factory to hide the construction of the CostFunction object from
//    // the client code.
//    static ceres::CostFunction* Create(const double observed_x,
//                                       const double observed_y) {
//        return (new ceres::AutoDiffCostFunction<ReprojectionError_2D, 2, 6, 3>(
//                new ReprojectionError_2D(observed_x, observed_y)));
//    }
//    static void setIntrinsics(double fx,double fy,double cx,double cy)
//    {
//        fx_ = fx;
//        fy_ = fy;
//        cx_ = cx;
//        cy_ = cy;
//    }
//    static double fx_;
//    static double fy_;
//    static double cx_;
//    static double cy_;
//
//    double observed_x;
//    double observed_y;
//};

struct GeoError
{
    GeoError(const Eigen::Vector3d vs, const Eigen::Vector3d vt, const Eigen::Vector3d nt)
            : vs_(vs), vt_(vt), nt_(nt) {}

    template <typename T>
    bool operator()(const T* const pose_s,
                    const T* const pose_t,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,3> R_s,R_t,R_s_inv,R_t_inv;
        Eigen::Matrix<T,3,1> t_s,t_t;
        ceres::AngleAxisToRotationMatrix(pose_s,&R_s_inv(0));
        ceres::AngleAxisToRotationMatrix(pose_t,&R_t_inv(0));
        t_s << pose_s[3],pose_s[4],pose_s[5];
        t_t << pose_t[3],pose_t[4],pose_t[5];

        R_s = R_s_inv.transpose();
        R_t = R_t_inv.transpose();

        t_s = - R_s * t_s;
        t_t = - R_t * t_t;

        Eigen::Matrix<T,3,1> vs =  (Eigen::Matrix<T,3,1>() << T(vs_[0]),T(vs_[1]),T(vs_[2])).finished();
        Eigen::Matrix<T,3,1> vt =  (Eigen::Matrix<T,3,1>() << T(vt_[0]),T(vt_[1]),T(vt_[2])).finished();
        Eigen::Matrix<T,3,1> nt =  (Eigen::Matrix<T,3,1>() << T(nt_[0]),T(nt_[1]),T(nt_[2])).finished();

        vs = R_s * vs;
        vs += t_s;

        vt = R_t * vt;
        vt += t_t;

        residuals[0] = vs[0] - vt[0];
        residuals[1] = vs[1] - vt[1];
        residuals[2] = vs[2] - vt[2];

//        residuals[0] = (vs - vt).dot(nt);
//        if(residuals[0] > T (0.025))
//        {
//            std::cout<<"norm over"<<std::endl;
//        }
        return true;
    }

    static ceres::CostFunction* Create(
            const Eigen::Vector3d vs,
            const Eigen::Vector3d vt,
            const Eigen::Vector3d nt) {
        return (new ceres::AutoDiffCostFunction<GeoError, 3, 6, 6>(
                new GeoError(vs, vt, nt)));
    }

    Eigen::Vector3d vs_;
    Eigen::Vector3d vt_;
    Eigen::Vector3d nt_;
};


struct AlignmentError_world
{
    AlignmentError_world(double u,double v, double d,
                         double u_ref, double v_ref, double d_ref):
                         u_(u),v_(v),d_(d),
                         u_ref_(u_ref),v_ref_(v_ref),d_ref_(d_ref){}
    template <typename T>
    bool operator()(const T* const intrinsics,
                    const T* const camera,
                    const T* const camera_ref,
                    T* residuals) const
    {
        T p_tmp[3];
        T p_obs[3];
        T p_ref[3];
        const T& fx_ = T (intrinsics[0]);
        const T& fy_ = T (intrinsics[1]);
        const T& cx_ = T (intrinsics[2]);
        const T& cy_ = T (intrinsics[3]);
        p_tmp[0] = (u_-cx_)*d_/fx_;
        p_tmp[1] = (v_-cy_)*d_/fy_;
        p_tmp[2] = T (d_);
        ceres::AngleAxisRotatePoint(camera, p_tmp, p_obs);
        p_obs[0] += camera[3];
        p_obs[1] += camera[4];
        p_obs[2] += camera[5];

        p_tmp[0] = (u_ref_-cx_)*d_ref_/fx_;
        p_tmp[1] = (v_ref_-cy_)*d_ref_/fy_;
        p_tmp[2] = T (d_ref_);
        ceres::AngleAxisRotatePoint(camera_ref, p_tmp, p_ref);
        p_ref[0] += camera_ref[3];
        p_ref[1] += camera_ref[4];
        p_ref[2] += camera_ref[5];


        residuals[0] = (p_ref[0] - p_obs[0]);
        residuals[1] = (p_ref[1] - p_obs[1]);
        residuals[2] = (p_ref[2] - p_obs[2]);

        T norm = residuals[0]*residuals[0] + residuals[1]*residuals[1] + residuals[2]*residuals[2];

        if(norm > T(0.25))
        {
            printf("norm over threshold\n");

        }

        return true;
    }
    static ceres::CostFunction* Create(const double u,
                                       const double v,
                                       const double d,
                                       const double u_ref,
                                       const double v_ref,
                                       const double d_ref)
    {
        return (new ceres::AutoDiffCostFunction<AlignmentError_world, 3, 4, 6, 6>(
                new AlignmentError_world(u, v, d, u_ref, v_ref, d_ref)));
    }
    double u_,v_,d_;
    double u_ref_,v_ref_,d_ref_;
};

struct AlignmentError
{
    AlignmentError(double observed_u, double observed_v, double observed_d):
                     u_(observed_u), v_(observed_v), d_(observed_d){}
    template <typename T>
    bool operator()(const T* const intrinsics,
                    const T* const camera,
                    const T* const point,
                    T* residuals) const
    {
        T p[3];
        T p_obs[3];

        ceres::AngleAxisRotatePoint(camera, point, p);

        const T& fx_ = T (intrinsics[0]);
        const T& fy_ = T (intrinsics[1]);
        const T& cx_ = T (intrinsics[2]);
        const T& cy_ = T (intrinsics[3]);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        p_obs[0] = (u_-cx_)*d_/fx_;
        p_obs[1] = (v_-cy_)*d_/fy_;
        p_obs[2] = T (d_);
        residuals[0] = (p[0] - p_obs[0]);
        residuals[1] = (p[1] - p_obs[1]);
        residuals[2] = (p[2] - p_obs[2]);

        T norm = residuals[0]*residuals[0] + residuals[1]*residuals[1] + residuals[2]*residuals[2];

        if(norm > T(0.25))
        {
            printf("norm over threshold\n");

        }
        return true;
    }
    static ceres::CostFunction* Create(const double observed_u,
                                       const double observed_v,
                                       const double observed_d) {
        return (new ceres::AutoDiffCostFunction<AlignmentError, 3, 4, 6, 3>(
                new AlignmentError(observed_u, observed_v,observed_d)));
    }

    double u_;
    double v_;
    double d_;
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

        ceres::AngleAxisRotatePoint(camera, point, p);


        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        const T& fx_ = T (intrinsics[0]);
        const T& fy_ = T (intrinsics[1]);
        const T& cx_ = T (intrinsics[2]);
        const T& cy_ = T (intrinsics[3]);

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

        if(sqrt(norm) > T(0.5))
        {
            printf("norm over threshold\n");
//            residuals[0] = T(0);
//            residuals[1] = T(0);
//            residuals[2] = T(0);
        }
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


//double ReprojectionError_2D::fx_;
//double ReprojectionError_2D::fy_;
//double ReprojectionError_2D::cx_;
//double ReprojectionError_2D::cy_;

#endif //BAMAPPING_ERROR_H
