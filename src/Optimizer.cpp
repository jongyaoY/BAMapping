//
// Created by jojo on 06.12.19.
//

#include "Optimizer.h"
#include "io/Parser.h"

using namespace BAMapping;
using namespace ceres;

void Optimizer::localGraphOptimize(Graph *pLocalGraph)
{
    Problem problem;
    Solver::Options options;
    double** cam_param;
    double** point_param;
    int camera_size = pLocalGraph->getFrameVectorSize();
    int point_size = pLocalGraph->getPointVectorSize();
    auto cameraBlock_size = pLocalGraph->getFrameBlockSize();
    auto pointBlock_size = pLocalGraph->getPointBlockSize();

    cam_param = new double* [camera_size];
    point_param = new double* [point_size];
    for(int i = 0; i < camera_size; i++)
    {
        cam_param[i] = new double[cameraBlock_size];
    }
    for(int i = 0; i < point_size; i++)
    {
        point_param[i] = new double[pointBlock_size];
    }

    builProblem(pLocalGraph,&problem,cam_param,point_param);
    SetMinimizerOptions(&options);
    SetLinearSolver(&options);

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    pLocalGraph->update(cam_param,point_param);

    std::cout << summary.FullReport() << "\n";
}

void Optimizer::builProblem(Graph* pGraph, Problem* problem,double** cam_param,double** point_param)
{
//    FrameVector frames = pGraph->getConstFrames();
//    PointVector points = pGraph->getConstPoints();

    int camera_size = pGraph->getFrameVectorSize();
    int point_size = pGraph->getPointVectorSize();
//    auto cameraBlock_size = pGraph->getFrameBlockSize();
//    auto pointBlock_size = pGraph->getPointBlockSize();

//    cam_param = new double* [camera_size];
//    point_param = new double* [point_size];
//    for(int i = 0; i < camera_size; i++)
//    {
//        cam_param[i] = new double[cameraBlock_size];
//    }
//    for(int i = 0; i < point_size; i++)
//    {
//        point_param[i] = new double[pointBlock_size];
//    }
    pGraph->getOptParameters(cam_param,point_param);
    auto edges = pGraph->getEdges();
    for(auto edge : edges)
    {
        auto Cam_Point = edge.first;
        auto observation = edge.second;
        auto cam_id = Cam_Point.first;
        auto point_id = Cam_Point.second;
        if(cam_id>=camera_size || point_id>=point_size)
        {
            std::cout<<"cam_id or point_id exceeds\n";
            continue;
        }

        CostFunction* cost_function;
        LossFunction* loss_function = new HuberLoss(1.0);

        cost_function = AlignmentError_3D::Create(observation[0], observation[1], observation[2]);

        problem->AddResidualBlock(cost_function, loss_function, cam_param[cam_id], point_param[point_id]);

    }
}

void Optimizer::SetMinimizerOptions(Solver::Options* options)
{
    options->max_num_iterations = 1000;
    options->minimizer_progress_to_stdout = true;
    options->num_threads = 6;
    options->eta = 1e-2;
    options->max_solver_time_in_seconds = 1e32;
    options->use_nonmonotonic_steps = false;

    options->minimizer_type = ceres::TRUST_REGION;

    options->trust_region_strategy_type = LEVENBERG_MARQUARDT;
    options->dogleg_type = SUBSPACE_DOGLEG;//TRADITIONAL_DOGLEG;

    options->use_inner_iterations = false;
}

void Optimizer::SetLinearSolver(Solver::Options* options)
{
    options->linear_solver_type = LinearSolverType::DENSE_SCHUR;
  options->preconditioner_type = PreconditionerType::JACOBI;
  options->visibility_clustering_type = VisibilityClusteringType::CANONICAL_VIEWS;
  options->sparse_linear_algebra_library_type = SparseLinearAlgebraLibraryType::SUITE_SPARSE;
  options->dense_linear_algebra_library_type = DenseLinearAlgebraLibraryType::EIGEN;
    options->use_explicit_schur_complement = false;
}

void Optimizer::init(std::string configFile)
{
    BAMapping::io::Parser parser;
    parser.load(configFile);
    double fx = parser.getValue<double>("Camera.fx");
    double fy = parser.getValue<double>("Camera.fy");
    double cx = parser.getValue<double>("Camera.cx");
    double cy = parser.getValue<double>("Camera.cy");
    AlignmentError_3D::setIntrinsics(fx,fy,cx,cy);
}
