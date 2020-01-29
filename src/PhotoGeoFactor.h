//
// Created by jojo on 26.01.20.
//

#ifndef BAMAPPING_PHOTOGEOFACTOR_H
#define BAMAPPING_PHOTOGEOFACTOR_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "Open3D/Geometry/PointCloud.h"
#include "Open3D/Registration/Registration.h"
#include "Open3D/Registration/ColoredICP.h"
#include "Open3D/Geometry/KDTreeFlann.h"
#include "Open3D/Geometry/KDTreeSearchParam.h"

#include "Eigen/Geometry"

namespace BAMapping
{
    using namespace open3d;
    using namespace open3d::registration;
    class PhotoGeoFactor : public ceres::SizedCostFunction<2, 6, 6>
    {
    public:
        PhotoGeoFactor() = delete;
        PhotoGeoFactor(geometry::PointCloud source,geometry::PointCloud target)
        : source_(source),target_(target){}
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            Eigen::Vector6d pose_s =  (Eigen::Vector6d() << parameters[0][0], parameters[0][1], parameters[0][2],
                                                            parameters[0][3], parameters[0][4], parameters[0][5]).finished();
            Eigen::Vector6d pose_t = (Eigen::Vector6d() <<  parameters[1][0], parameters[1][1], parameters[1][2],
                                                            parameters[1][3], parameters[1][4], parameters[1][5]).finished();
            Eigen::Matrix3d C_s,C_t;
            Eigen::Vector3d t_s, t_t;
            ceres::AngleAxisToRotationMatrix(pose_s.data(),C_s.data());
            ceres::AngleAxisToRotationMatrix(pose_t.data(),C_t.data());
            t_s = pose_s.tail(3);
            t_t = pose_t.tail(3);

            Eigen::Matrix4d Tcsw, Tctw, Tctcs;
            Tcsw = Eigen::Matrix4d::Identity();
            Tctw = Eigen::Matrix4d::Identity();

            Tctcs = Tctw * Tcsw.inverse();
            Eigen::Affine3d Tctcs_affine;
            Tctcs_affine.matrix() = Tctcs;

            geometry::KDTreeFlann kdtree;
            kdtree.SetGeometry(target_);

            auto result = GetRegistrationResultAndCorrespondences(source_,target_,kdtree,0.05,Tctcs);
            auto target_c = InitializePointCloudForColoredICP(
                    target_, geometry::KDTreeSearchParamHybrid(0.05 * 2.0, 30));

            double lambda_geometric_ = 0.6; //todo
            double sqrt_lambda_geometric = sqrt(lambda_geometric_);
            double lambda_photometric = 1.0 - lambda_geometric_;
            double sqrt_lambda_photometric = sqrt(lambda_photometric);

            std::vector<Eigen::Matrix<double,12,2>> J_r;
            std::vector<Eigen::Vector2d> r;
            for(size_t i = 0; i < result.correspondence_set_.size(); i++)
            {
                auto corres = result.correspondence_set_[i];
                size_t cs = corres[0];
                size_t ct = corres[1];

                const Eigen::Vector3d &vs = Tctcs_affine * source_.points_[cs]; //todo
                const Eigen::Vector3d &vt = target_.points_[ct];
                const Eigen::Vector3d &nt = target_.normals_[ct];

                J_r[i].block<3, 1>(0, 0) = sqrt_lambda_geometric * vs.cross(nt);
                J_r[i].block<3, 1>(3, 0) = sqrt_lambda_geometric * nt;
                r[i][0] = sqrt_lambda_geometric * (vs - vt).dot(nt);

                // project vs into vt's tangential plane
                Eigen::Vector3d vs_proj = vs - (vs - vt).dot(nt) * nt;
                double is = (source_.colors_[cs](0) + source_.colors_[cs](1) +
                             source_.colors_[cs](2)) /
                            3.0;
                double it = (target_.colors_[ct](0) + target_.colors_[ct](1) +
                             target_.colors_[ct](2)) /
                            3.0;
                const Eigen::Vector3d &dit = target_c->color_gradient_[ct];
                double is0_proj = (dit.dot(vs_proj - vt)) + it;

                const Eigen::Matrix3d M =
                        (Eigen::Matrix3d() << 1.0 - nt(0) * nt(0),
                                -nt(0) * nt(1), -nt(0) * nt(2), -nt(0) * nt(1),
                                1.0 - nt(1) * nt(1), -nt(1) * nt(2), -nt(0) * nt(2),
                                -nt(1) * nt(2), 1.0 - nt(2) * nt(2))
                                .finished();
                const Eigen::Vector3d &ditM = -dit.transpose() * M;
                J_r[i].block<3, 1>(0, 1) =
                        sqrt_lambda_photometric * vs.cross(ditM);
                J_r[i].block<3, 1>(3, 1) = sqrt_lambda_photometric * ditM;
                r[i][1] = sqrt_lambda_photometric * (is - is0_proj);
            }


            return true;
        }

    private:
        geometry::PointCloud source_;
        geometry::PointCloud target_;


    private:
        class PointCloudForColoredICP : public geometry::PointCloud {
        public:
            std::vector<Eigen::Vector3d> color_gradient_;
        };


        std::shared_ptr<PointCloudForColoredICP> InitializePointCloudForColoredICP(
                const geometry::PointCloud &target,
                const geometry::KDTreeSearchParamHybrid &search_param) const {

            geometry::KDTreeFlann tree;
            tree.SetGeometry(target);

            auto output = std::make_shared<PointCloudForColoredICP>();
            output->colors_ = target.colors_;
            output->normals_ = target.normals_;
            output->points_ = target.points_;

            size_t n_points = output->points_.size();
            output->color_gradient_.resize(n_points, Eigen::Vector3d::Zero());

            for (size_t k = 0; k < n_points; k++) {
                const Eigen::Vector3d &vt = output->points_[k];
                const Eigen::Vector3d &nt = output->normals_[k];
                double it = (output->colors_[k](0) + output->colors_[k](1) +
                             output->colors_[k](2)) /
                            3.0;

                std::vector<int> point_idx;
                std::vector<double> point_squared_distance;

                if (tree.SearchHybrid(vt, search_param.radius_, search_param.max_nn_,
                                      point_idx, point_squared_distance) >= 4) {
                    // approximate image gradient of vt's tangential plane
                    size_t nn = point_idx.size();
                    Eigen::MatrixXd A(nn, 3);
                    Eigen::MatrixXd b(nn, 1);
                    A.setZero();
                    b.setZero();
                    for (size_t i = 1; i < nn; i++) {
                        int P_adj_idx = point_idx[i];
                        Eigen::Vector3d vt_adj = output->points_[P_adj_idx];
                        Eigen::Vector3d vt_proj = vt_adj - (vt_adj - vt).dot(nt) * nt;
                        double it_adj = (output->colors_[P_adj_idx](0) +
                                         output->colors_[P_adj_idx](1) +
                                         output->colors_[P_adj_idx](2)) /
                                        3.0;
                        A(i - 1, 0) = (vt_proj(0) - vt(0));
                        A(i - 1, 1) = (vt_proj(1) - vt(1));
                        A(i - 1, 2) = (vt_proj(2) - vt(2));
                        b(i - 1, 0) = (it_adj - it);
                    }
                    // adds orthogonal constraint
                    A(nn - 1, 0) = (nn - 1) * nt(0);
                    A(nn - 1, 1) = (nn - 1) * nt(1);
                    A(nn - 1, 2) = (nn - 1) * nt(2);
                    b(nn - 1, 0) = 0;
                    // solving linear equation
                    bool is_success;
                    Eigen::MatrixXd x;
                    std::tie(is_success, x) = utility::SolveLinearSystemPSD(
                            A.transpose() * A, A.transpose() * b);
                    if (is_success) {
                        output->color_gradient_[k] = x;
                    }
                }
            }
            return output;
        }

        RegistrationResult GetRegistrationResultAndCorrespondences(
                const geometry::PointCloud &source,
                const geometry::PointCloud &target,
                const geometry::KDTreeFlann &target_kdtree,
                double max_correspondence_distance,
                const Eigen::Matrix4d &transformation) const
        {
            RegistrationResult result(transformation);
            if (max_correspondence_distance <= 0.0) {
                return result;
            }

            double error2 = 0.0;

#ifdef _OPENMP
#pragma omp parallel
            {
#endif
                double error2_private = 0.0;
                CorrespondenceSet correspondence_set_private;
#ifdef _OPENMP
#pragma omp for nowait
#endif
                for (int i = 0; i < (int)source.points_.size(); i++) {
                    std::vector<int> indices(1);
                    std::vector<double> dists(1);
                    const auto &point = source.points_[i];
                    if (target_kdtree.SearchHybrid(point, max_correspondence_distance,
                                                   1, indices, dists) > 0) {
                        error2_private += dists[0];
                        correspondence_set_private.push_back(
                                Eigen::Vector2i(i, indices[0]));
                    }
                }
#ifdef _OPENMP
#pragma omp critical
#endif
                {
                    for (int i = 0; i < (int)correspondence_set_private.size(); i++) {
                        result.correspondence_set_.push_back(
                                correspondence_set_private[i]);
                    }
                    error2 += error2_private;
                }
#ifdef _OPENMP
            }
#endif

            if (result.correspondence_set_.empty()) {
                result.fitness_ = 0.0;
                result.inlier_rmse_ = 0.0;
            } else {
                size_t corres_number = result.correspondence_set_.size();
                result.fitness_ = (double)corres_number / (double)source.points_.size();
                result.inlier_rmse_ = std::sqrt(error2 / (double)corres_number);
            }
            return result;
        }
    };
}



#endif //BAMAPPING_PHOTOGEOFACTOR_H
