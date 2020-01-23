//
// Created by jojo on 22.01.20.
//

#include "BundleAdjuster.h"

using namespace BAMapping;

void BAMapping::BundleAdjuster::optimize(BAMapping::Graph &graph, const char* config_file)
{
    Parser config(config_file);
    ceres::Problem problem;
    auto extrinsics = packCameraParam(graph);
    auto points = packPointParam(graph);
    auto intrinsics = getIntrinsics(config, graph.nodes_.size());

    bool is_camera_locked = false;
    AlignmentError_3D::setIntrinsics(431.828094,431.828094,323.000610,240.218506);
    for(auto edge : graph.edges_)
    {
        auto cam_id = edge.node_id_;
        auto point_id = edge.point_id_;
        auto obs = edge.obs_;

        ceres::CostFunction* cost_function = ReprojectionError_3D::Create(obs[0],obs[1],obs[2]);
        problem.AddResidualBlock(cost_function,NULL, &intrinsics[cam_id](0), &extrinsics[cam_id](0), &points[point_id](0));

//        ceres::CostFunction* cost_function = AlignmentError_3D::Create(obs[0],obs[1],obs[2]);
//        problem.AddResidualBlock(cost_function,NULL, &extrinsics[cam_id](0), &points[point_id](0));

        if (!is_camera_locked)
        {
            problem.SetParameterBlockConstant(&extrinsics[0](0));
            is_camera_locked = true;
        }
//        problem.SetParameterBlockConstant(&intrinsics[cam_id](0));

    }

    for(int i = 0; i < intrinsics.size(); ++i)
        problem.SetParameterBlockConstant(&intrinsics[i](0));



    ceres::Solver::Options options;

//    options.minimizer_progress_to_stdout = true;
//    options.num_threads = 1;
//    options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
//    options.preconditioner_type = ceres::PreconditionerType::JACOBI;
//    options.use_nonmonotonic_steps = false;
//    options.minimizer_type = ceres::TRUST_REGION;
//    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
//    options.dogleg_type = ceres::SUBSPACE_DOGLEG;//TRADITIONAL_DOGLEG;


    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.use_inner_iterations = true;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    unpackCameraParam(graph,extrinsics);
    unpackPointParam(graph,points);
    std::cout << summary.FullReport() << std::endl;
}

std::vector<Vec6> BundleAdjuster::packCameraParam(const Graph& graph)
{
    using namespace std;
    vector<Vec6> extrinsics(graph.nodes_.size());
    for (int i = 0; i < graph.nodes_.size(); i++)
    {
        auto Tcw = graph.nodes_[i].pose_.inverse();
        Mat3 R = Tcw.block<3, 3>(0, 0).cast<double>();
        ceres::RotationMatrixToAngleAxis(&R(0, 0), &extrinsics[i](0));

        extrinsics[i].tail<3>() = Tcw.block<3, 1>(0, 3);
    }

    return extrinsics;
}

std::vector<Vec3> BundleAdjuster::packPointParam(const Graph& graph)
{
    using namespace std;
    vector<Vec3> pts3d(graph.points_.size());
    for (int i = 0; i < graph.points_.size(); i++)
    {
        pts3d[i] = graph.points_[i].pose_.cast<double>();
    }

    return pts3d;
}

void BundleAdjuster::unpackCameraParam(Graph &graph, const std::vector<Vec6> &extrinsics)
{
    for (int i = 0; i < graph.nodes_.size(); i++)
    {
        Mat3 R;
        Vec3 t;
        ceres::AngleAxisToRotationMatrix(&extrinsics[i](0), &R(0, 0));
        t = extrinsics[i].tail<3>().cast<double>();

        Mat3 R_inv = R.transpose();
        t = -R_inv * t;

        graph.nodes_[i].pose_.block<3, 3>(0, 0) = R_inv.cast<double>();
        graph.nodes_[i].pose_.block<3, 1>(0, 3) = t;
    }
}

std::vector<Vec4> BundleAdjuster::getIntrinsics(Parser config, size_t n)
{
    using namespace std;
    double fx,fy,cx,cy;
    fx = config.getValue<double>("Camera.fx");
    fy = config.getValue<double>("Camera.fy");
    cx = config.getValue<double>("Camera.cx");
    cy = config.getValue<double>("Camera.cy");

    vector<Vec4> intrinsics(n);

    for(size_t i = 0; i < n; i++)
    {
        intrinsics[i] = Vec4(fx,fy,cx,cy);
    }
    return intrinsics;
}

void BundleAdjuster::unpackPointParam(Graph &graph, const std::vector<Vec3> &points)
{
    for (int i = 0; i < graph.points_.size(); i++)
    {
        graph.points_[i].pose_ = points[i].cast<double>();
    }
}
