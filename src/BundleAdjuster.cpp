//
// Created by jojo on 22.01.20.
//

#include "BundleAdjuster.h"
//#include "imu_factor.h"
#include "PhotoGeoFactor.h"
#include "GeometryMethods.h"

using namespace BAMapping;

void BAMapping::BundleAdjuster::optimize(BAMapping::Graph &graph, const char* config_file)
{
    Parser config(config_file);
    ceres::Problem problem;
    auto extrinsics = packCameraParam(graph);
    auto points = packPointParam(graph);
    auto intrinsics = getIntrinsics(config, graph.nodes_.size());
    bool dense_term = config.getValue<bool>("use_dense_term");
    bool is_camera_locked = false;
    for(auto edge : graph.edges_)
    {
        auto cam_id = edge.node_id_;
        auto point_id = edge.point_id_;
        auto obs = edge.obs_;

        ceres::CostFunction* cost_function = ReprojectionError_3D::Create(obs[0],obs[1],obs[2]);
        problem.AddResidualBlock(cost_function,NULL, &intrinsics[cam_id](0), &extrinsics[cam_id](0), &points[point_id](0));


        if (!is_camera_locked)
        {
            problem.SetParameterBlockConstant(&extrinsics[0](0));
            is_camera_locked = true;
        }
    }

    for(int i = 0; i < intrinsics.size(); ++i)
        problem.SetParameterBlockConstant(&intrinsics[i](0));

    if(dense_term)
    {
        double voxel_size = config.getValue<double>("voxel_size");
        std::shared_ptr<open3d::geometry::PointCloud> source;
        std::shared_ptr<open3d::geometry::PointCloud> target;
        GeometryMethods::createPointCloundFromNodes({graph.nodes_[0]},config,source,true);
        auto source_down = source->VoxelDownSample(voxel_size);

        for(size_t i = 0; i < graph.nodes_.size()-1; i++)
        {

            auto node_s = graph.nodes_[i];
            auto node_t = graph.nodes_[i + 1];


            bool sucess = false;
//        sucess = GeometryMethods::createPointCloundFromNodes({node_s},config,source,true);
//        if(!sucess)
//            continue;
            sucess = GeometryMethods::createPointCloundFromNodes({node_t},config,target,true);
            if(!sucess)
                continue;

            auto target_down = target->VoxelDownSample(voxel_size);

            target_down->EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size*2.0,30));

            auto result = GeoFactor_single::GetRegistrationResultAndCorrespondences(*source_down,*target_down,voxel_size,node_t.pose_.inverse()*node_s.pose_);
            for(auto corr : result.correspondence_set_)
            {
                auto s = corr[0];
                auto t = corr[1];
                ceres::CostFunction* cost_function = GeoError::Create(source_down->points_[s],target_down->points_[t],target_down->normals_[t]);
//            ceres::ScaledLoss* weight = new ceres::ScaledLoss(NULL,1,ceres::Ownership::DO_NOT_TAKE_OWNERSHIP);
                problem.AddResidualBlock(cost_function,NULL,&extrinsics[i](0),&extrinsics[i + 1](0));
            }

            *source_down = *target_down;
            if (!is_camera_locked)
            {
                problem.SetParameterBlockConstant(&extrinsics[0](0));
                is_camera_locked = true;
            }
        }
    }


    ceres::Solver::Options options;

//    options.use_nonmonotonic_steps = true;
//    options.preconditioner_type = ceres::SCHUR_JACOBI;
//    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
//    options.use_inner_iterations = true;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;//TRADITIONAL_DOGLEG;
    options.max_num_iterations = 30;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    unpackCameraParam(graph,extrinsics);
    unpackPointParam(graph,points);
    std::cout << summary.FullReport() << std::endl;
}

void BundleAdjuster::optimizeGlobal(Graph &graph, const char *config_file, const std::vector<std::string>& plyNames)
{
    Parser config(config_file);
    ceres::Problem problem;
    auto extrinsics = packCameraParam(graph);
    auto points = packPointParam(graph);
    double sparse_weight = config.getValue<double>("global_sparse_weight");
    double dense_weight = 1 - sparse_weight;
    bool is_camera_locked = false;
    for(auto edge : graph.edges_)
    {
        auto cam_id = edge.node_id_;
        auto point_id = edge.point_id_;
        auto obs = edge.obs_;


        ceres::CostFunction* cost_function = AlignmentError_3D_Direct::Create(obs[0],obs[1],obs[2]);
        ceres::ScaledLoss* weight = new ceres::ScaledLoss(NULL,sparse_weight,ceres::Ownership::DO_NOT_TAKE_OWNERSHIP);

        problem.AddResidualBlock(cost_function,NULL, &extrinsics[cam_id](0), &points[point_id](0));

        if (!is_camera_locked)
        {
            problem.SetParameterBlockConstant(&extrinsics[0](0));
            is_camera_locked = true;
        }
    }
    if(!plyNames.empty())
    {
        double voxel_size = config.getValue<double>("voxel_size");
        for(size_t i = 0; i < graph.nodes_.size()-1; i++)
        {

            auto node_s = graph.nodes_[i];
            auto node_t = graph.nodes_[i + 1];
            std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
            std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
            geometry::TriangleMesh source_ply;
            geometry::TriangleMesh target_ply;

            io::ReadPointCloud(plyNames[i],*source);
            io::ReadPointCloud(plyNames[i+1],*target);
//            io::ReadTriangleMeshFromPLY(plyNames[i],source_ply,false);
//            io::ReadTriangleMeshFromPLY(plyNames[i+1],target_ply,false);


            auto source_down = source->VoxelDownSample(voxel_size);
            auto target_down = target->VoxelDownSample(voxel_size);

            target_down->EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size*2.0,30));

            auto result = GeoFactor_single::GetRegistrationResultAndCorrespondences(*source_down,*target_down,voxel_size,node_t.pose_.inverse()*node_s.pose_);

            for(auto corr : result.correspondence_set_)
            {
                auto s = corr[0];
                auto t = corr[1];
                ceres::CostFunction* cost_function = GeoError::Create(source_down->points_[s],target_down->points_[t],target_down->normals_[t]);
                ceres::ScaledLoss* weight = new ceres::ScaledLoss(NULL,dense_weight,ceres::Ownership::DO_NOT_TAKE_OWNERSHIP);
                problem.AddResidualBlock(cost_function, weight,&extrinsics[i](0),&extrinsics[i + 1](0));
            }

            if (!is_camera_locked)
            {
                problem.SetParameterBlockConstant(&extrinsics[0](0));
                is_camera_locked = true;
            }
        }
    }


    ceres::Solver::Options options;

    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.use_inner_iterations = true;
    options.max_num_iterations = 30;
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
