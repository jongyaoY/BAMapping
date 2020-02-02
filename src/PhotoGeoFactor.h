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
#include "Open3D/Visualization/Utility/DrawGeometry.h"
#include "Eigen/Geometry"

namespace BAMapping
{
    using namespace open3d;
    using namespace open3d::registration;
    using namespace open3d::geometry;
    class PhotoGeoFactor : public ceres::SizedCostFunction<2, 6, 6>
    {
    public:
        PhotoGeoFactor() = delete;
        PhotoGeoFactor(geometry::PointCloud source, geometry::PointCloud target, const double voxel_size, const double lambda_geometric)
        : source_(source),target_(target),voxel_size_(voxel_size),lambda_geometric_(lambda_geometric){}
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

            Tcsw.block<3,3>(0,0) = C_s;
            Tctw.block<3,3>(0,0) = C_t;
            Tcsw.block<3,1>(0,3) = t_s;
            Tctw.block<3,1>(0,3) = t_t;

            Tctcs = Tctw * Tcsw.inverse();
            Eigen::Affine3d Tctcs_affine;
            Tctcs_affine.matrix() = Tctcs;

            geometry::KDTreeFlann kdtree;
            kdtree.SetGeometry(target_);
            geometry::PointCloud source_transformed = source_;
            source_transformed.Transform(Tctcs);
            auto result = GetRegistrationResultAndCorrespondences(source_transformed ,target_,kdtree,voxel_size_,Tctcs);
            auto target_c = InitializePointCloudForColoredICP(
                    target_, geometry::KDTreeSearchParamHybrid(voxel_size_ * 2.0, 30));

            double sqrt_lambda_geometric = sqrt(lambda_geometric_);
            double lambda_photometric = 1.0 - lambda_geometric_;
            double sqrt_lambda_photometric = sqrt(lambda_photometric);

            std::vector<Eigen::Matrix<double, 6, 2>> J_r;
            std::vector<Eigen::Vector2d> r;
            J_r.reserve(result.correspondence_set_.size());
            r.reserve(result.correspondence_set_.size());
            for(size_t i = 0; i < result.correspondence_set_.size(); i++)
            {
                auto corres = result.correspondence_set_[i];
                size_t cs = corres[0];
                size_t ct = corres[1];

                const Eigen::Vector3d &vs = Tctcs_affine * source_.points_[cs]; //todo
                const Eigen::Vector3d &vt = target_.points_[ct];
                const Eigen::Vector3d &nt = target_.normals_[ct];

                J_r[i].block<3, 1>(0, 0) = sqrt_lambda_geometric * vs.cross(nt); //todo
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
                        (Eigen::Matrix3d() <<
                                1.0 - nt(0) * nt(0),  -nt(0) * nt(1), -nt(0) * nt(2),
                                -nt(0) * nt(1), 1.0 - nt(1) * nt(1), -nt(1) * nt(2),
                                -nt(0) * nt(2), -nt(1) * nt(2), 1.0 - nt(2) * nt(2))
                                .finished();
                const Eigen::Vector3d &ditM = -dit.transpose() * M;

                J_r[i].block<3, 1>(0, 1) =
                        sqrt_lambda_photometric * vs.cross(ditM); //todo
                J_r[i].block<3, 1>(3, 1) = sqrt_lambda_photometric * ditM;
                r[i][1] = sqrt_lambda_photometric * (is - is0_proj);
            }

            if(residuals)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 1>> residual(residuals);
                residual.setZero();
                for(size_t i = 0; i < result.correspondence_set_.size(); i++)
                {
                    residual[0] += r[i][0] * r[i][0];
                    residual[1] += r[i][1] * r[i][1];

                }
            }
            if(jacobians)
            {

                if(jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jacobian_pose_s(jacobians[0]);
                    jacobian_pose_s.setZero();
                    for(size_t i = 0; i < result.correspondence_set_.size(); i++)
                    {
                        jacobian_pose_s += J_r[i].transpose();
                    }
                }
                if(jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jacobian_pose_t(jacobians[1]);
                    jacobian_pose_t.setZero();
                }
            }

            return true;
        }

    private:
        geometry::PointCloud source_;
        geometry::PointCloud target_;
        double voxel_size_;
        double lambda_geometric_;


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

    class GeoFactor : public ceres::SizedCostFunction<1, 6, 6>
    {

    public:
        GeoFactor() = delete;
        GeoFactor(geometry::PointCloud source, geometry::PointCloud target, const double voxel_size)
        : source_(source),target_(target),voxel_size_(voxel_size){}
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            PointCloud source_cur = source_;
            PointCloud target_cur = target_;
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

            Eigen::Matrix4d Tcsw, Tctw;
            Tcsw = Eigen::Matrix4d::Identity();
            Tctw = Eigen::Matrix4d::Identity();

            Tcsw.block<3,3>(0,0) = C_s;
            Tcsw.block<3,1>(0,3) = t_s;

            Tctw.block<3,3>(0,0) = C_t;
            Tctw.block<3,1>(0,3) = t_t;

            source_cur.Transform(Tcsw.inverse());
            target_cur.Transform(Tctw.inverse());

//            std::cout<<Tcsw<<"\n"<<std::endl;
//            std::cout<<Tctw<<"\n"<<std::endl;

            KDTreeFlann kdtree;
            kdtree.SetGeometry(target_cur);

            auto result = GetRegistrationResultAndCorrespondences(source_cur,target_cur,kdtree,voxel_size_,Tctw*Tcsw.inverse());

            std::vector<Eigen::Matrix<double, 1, 6>> Js;
            std::vector<Eigen::Matrix<double, 1, 6>> Jt;

            std::vector<double> r;
            Js.reserve(result.correspondence_set_.size());
            Jt.reserve(result.correspondence_set_.size());
            r.reserve(result.correspondence_set_.size());
            for(size_t i = 0; i < result.correspondence_set_.size(); i++)
            {
                auto corres = result.correspondence_set_[i];
                size_t cs = corres[0];
                size_t ct = corres[1];

                const Eigen::Vector3d &vs = source_cur.points_[cs]; //todo
                const Eigen::Vector3d &vt = target_cur.points_[ct];
                const Eigen::Vector3d &nt = target_cur.normals_[ct];

                Js[i].block<1, 3>(0, 0) = (vs.cross(nt)).transpose();
                Js[i].block<1, 3>(0, 3) = nt.transpose();

                Jt[i].block<1, 3>(0, 0) = -(vt.cross(nt)).transpose();
                Jt[i].block<1, 3>(0, 3) = -nt.transpose();

                r[i] = (vs - vt).dot(nt);
            }
            if(residuals)
            {
                residuals[0] = 0;
                for(size_t i = 0; i < result.correspondence_set_.size(); i++)
                {
                    residuals[0] += 0.5 * r[i] * r[i];
//                    std::cout<<residuals[0]<<std::endl;

                }
            }
            if(jacobians)
            {
                if(jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jacobian_pose_s(jacobians[0]);
                    jacobian_pose_s.setZero();
                    for(size_t i = 0; i < result.correspondence_set_.size(); i++)
                    {
                        jacobian_pose_s += r[i] * Js[i];
                    }
                }
                if(jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jacobian_pose_t(jacobians[1]);
                    jacobian_pose_t.setZero();
                    for(size_t i = 0; i < result.correspondence_set_.size(); i++)
                    {
                        jacobian_pose_t += r[i] * Jt[i];
                    }
                }
            }
            return true;

        }
    private:
       PointCloud source_;
       PointCloud target_;
        double voxel_size_;


    private:
        RegistrationResult GetRegistrationResultAndCorrespondences(
                const PointCloud &source,
                const PointCloud &target,
                const KDTreeFlann &target_kdtree,
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

    class GeoFactor_single : public ceres::SizedCostFunction<1, 6, 6>
    {
    public:
        GeoFactor_single() = delete;
        ~GeoFactor_single()
        {
//            printf("des\n");
        }
        GeoFactor_single(const Eigen::Vector3d& vs, const Eigen::Vector3d& vt, const Eigen::Vector3d& nt):
        vs_(vs),vt_(vt),nt_(nt){};
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            Eigen::Vector6d pose_s =  (Eigen::Vector6d() << parameters[0][0], parameters[0][1], parameters[0][2],
                    parameters[0][3], parameters[0][4], parameters[0][5]).finished();
            Eigen::Vector6d pose_t = (Eigen::Vector6d() <<  parameters[1][0], parameters[1][1], parameters[1][2],
                    parameters[1][3], parameters[1][4], parameters[1][5]).finished();

            Eigen::Matrix3d C_s,C_t;
            Eigen::Vector3d t_s, t_t,nt;
            ceres::AngleAxisToRotationMatrix(pose_s.data(),C_s.data());
            ceres::AngleAxisToRotationMatrix(pose_t.data(),C_t.data());
            t_s = pose_s.tail(3);
            t_t = pose_t.tail(3);

            t_s = -C_s.transpose() * t_s;
            t_t = -C_t.transpose() * t_t;
            Eigen::Vector3d ps;
            Eigen::Vector3d pt;

            ps = C_s.transpose() * vs_;
            ps += t_s;

            pt = C_t.transpose() * vt_;
            pt += t_t;

            nt = C_t.transpose() * nt_;
            nt += t_t;


            Eigen::Matrix<double, 1, 6> Js;
            Eigen::Matrix<double, 1, 6> Jt;

            Js.block<1, 3>(0, 0) = (ps.cross(nt)).transpose();
            Js.block<1, 3>(0, 3) = nt.transpose();

            Jt.block<1, 3>(0, 0) = -(pt.cross(nt)).transpose();
            Jt.block<1, 3>(0, 3) = -nt.transpose();


//            std::cout<<ps<<""<<std::endl;
//            std::cout<<pt<<"\n"<<std::endl;
            residuals[0] = (ps - pt).dot(nt_);
            if(jacobians)
            {
                if(jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jacobian_pose_s(jacobians[0]);
                    jacobian_pose_s.setZero();
                    jacobian_pose_s += Js;
//                    std::cout<<"J1 "<<jacobians[0][0]<<" "<<jacobians[0][1]<<" "<<jacobians[0][2]<<" "<<jacobians[0][3]<<" "<<jacobians[0][4]<<" "<<jacobians[0][5]<<"\n"<<std::endl;
                }
                if(jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jacobian_pose_t(jacobians[1]);
                    jacobian_pose_t.setZero();
                    jacobian_pose_t += Jt;
                }

            }
        }

        static RegistrationResult GetRegistrationResultAndCorrespondences(
                const PointCloud &source_cs,
                const PointCloud &target,
                double max_correspondence_distance,
                const Eigen::Matrix4d &transformation)
        {
            RegistrationResult result(transformation);
            if (max_correspondence_distance <= 0.0) {
                return result;
            }

            double error2 = 0.0;
            const KDTreeFlann target_kdtree(target);
            geometry::PointCloud source = source_cs;
            source.Transform(transformation);
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

    private:
        Eigen::Vector3d vs_;
        Eigen::Vector3d vt_;
        Eigen::Vector3d nt_;
    };

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

            residuals[0] = (vs - vt).dot(nt);
            return true;
        }

        static ceres::CostFunction* Create(
                const Eigen::Vector3d vs,
                const Eigen::Vector3d vt,
                const Eigen::Vector3d nt) {
            return (new ceres::AutoDiffCostFunction<GeoError, 1, 6, 6>(
                    new GeoError(vs, vt, nt)));
        }

        Eigen::Vector3d vs_;
        Eigen::Vector3d vt_;
        Eigen::Vector3d nt_;
    };
}





#endif //BAMAPPING_PHOTOGEOFACTOR_H
