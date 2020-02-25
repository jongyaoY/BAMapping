//
// Created by jojo on 16.02.20.
//

#ifndef BAMAPPING_REPROJECTIONFACTOR_H
#define BAMAPPING_REPROJECTIONFACTOR_H
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "Eigen/Geometry"
#include "math/Types.h"
namespace BAMapping
{
class ReprojectionFactor : public ceres::SizedCostFunction<3,6,3>
{
public:
    ReprojectionFactor() = delete;
    ReprojectionFactor(Vec4 intrinsics, Vec3 obs):
    fx_(intrinsics[0]), fy_(intrinsics[1]),cx_(intrinsics[2]),cy_(intrinsics[3]),
    u_(obs[0]),v_(obs[1]),d_(obs[2]){}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Vec6 pose = (Vec6() << parameters[0][0], parameters[0][1], parameters[0][2],
                               parameters[0][3], parameters[0][4], parameters[0][5]).finished();
        Vec3 point = (Vec3() << parameters[1][0], parameters[1][1], parameters[1][2]).finished();

        Vec3 p;
        Vec3 p_obs;
        ceres::AngleAxisRotatePoint(&pose(0),&point(0),&p(0));
        p += pose.tail(3);
        p_obs[0] = (u_-cx_)*d_/fx_;
        p_obs[1] = (v_-cy_)*d_/fy_;
        p_obs[2] = d_;

        residuals[0] = (p[0] - p_obs[0]);
        residuals[1] = (p[1] - p_obs[1]);
        residuals[2] = (p[2] - p_obs[2]);

        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J1(jacobians[0]);
                J1.setZero();
                J1.block<3,3>(0,0) << 0, p[2], -p[1],
                                                       -p[2], 0 , p[0],
                                                       p[1], -p[0], 0;
                J1.block<3,3>(0,3) = Mat3::Identity();

            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J2(jacobians[1]);
                Mat3 R;
                ceres::AngleAxisToRotationMatrix(&pose(0),&R(0));
                J2 = R;
            }
        }

        return true;

    }
private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double u_,v_,d_;
};
}

#endif //BAMAPPING_REPROJECTIONFACTOR_H
