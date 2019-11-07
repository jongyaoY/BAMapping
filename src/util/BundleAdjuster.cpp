#include "BundleAdjuster.h"
#include "snavely_reprojection_error.h"




BundleAdjuster::BundleAdjuster()
{

}

void BundleAdjuster::SetLinearSolver(Solver::Options* options)
{
  options->linear_solver_type = LinearSolverType::DENSE_SCHUR;
//  options->preconditioner_type = PreconditionerType::JACOBI;
//  options->visibility_clustering_type = VisibilityClusteringType::CANONICAL_VIEWS;
//  options->sparse_linear_algebra_library_type = SparseLinearAlgebraLibraryType::SUITE_SPARSE;
//  options->dense_linear_algebra_library_type = DenseLinearAlgebraLibraryType::EIGEN;
  options->use_explicit_schur_complement = false;
}

void BundleAdjuster::SetMinimizerOptions(Solver::Options* options)
{
  options->max_num_iterations = 50;
  options->minimizer_progress_to_stdout = true;
  options->num_threads = 4;
  options->eta = 1e-2;
  options->max_solver_time_in_seconds = 1e32;
  options->use_nonmonotonic_steps = false;

  options->minimizer_type = ceres::TRUST_REGION;

  options->trust_region_strategy_type = LEVENBERG_MARQUARDT;
  options->dogleg_type = SUBSPACE_DOGLEG;//TRADITIONAL_DOGLEG;

  options->use_inner_iterations = false;
}

void BundleAdjuster::SetSolverOptions(Graph *graph, Solver::Options* options)
{
  SetMinimizerOptions(options);
  SetLinearSolver(options);
  SetOrdering(graph, options);
}

void BundleAdjuster::SetOrdering(Graph *graph, Solver::Options* options)
{
    ceres::ParameterBlockOrdering* ordering =
        new ceres::ParameterBlockOrdering;
    //first eliminate points. after solvine cameras,substitute back to solve points
    // The points come before the cameras.
    int point_size = graph->getPoints().size();
    for(int i = 0; i < point_size; i++)
    {
        ordering->AddElementToGroup(point_param + 3 * i, 0);
    }
    // the entire camera.

    // When using axis-angle, there is a single parameter block for
    int frame_size = graph->getFrames().size();
    for(int i = 0; i < frame_size; i++)
    {
        ordering->AddElementToGroup(cam_param + 9 * i, 1);
    }

    options->linear_solver_ordering.reset(ordering);
}

void BundleAdjuster::BuildProblem(Graph *graph,Problem* problem)
{
    FramePtrVector pFrames = graph->getFrames();
    PointPtrMap pPoints = graph->getPoints();

    int camera_size = 9*pFrames.size();
    int point_size = 3*pPoints.size();

    cam_param = new double[camera_size];
    point_param = new double[point_size];
    graph->getOptParameters(cam_param,point_param);

    for(int i = 0; i < pFrames.size(); i++)
    {
        Frame::ObservationVector observations = pFrames[i]->getObservations();

        for(int j = 0; j < observations.size(); j++)
        {
            CostFunction* cost_function;
            cost_function = Error::Create(observations[j].second[0], observations[j].second[1]);
            LossFunction* loss_function = new HuberLoss(1.0);
            problem->AddResidualBlock(cost_function, NULL, cam_param + 9*i, point_param+3*observations[j].first);
        }
    }
}

void BundleAdjuster::solve(Graph *graph)
{
    Problem problem;
    Solver::Options options;
    BuildProblem(graph,&problem);
    SetSolverOptions(graph,&options);
//    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
//    options.gradient_tolerance = 1e-16;
//    options.function_tolerance = 1e-16;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    graph->update(cam_param,point_param);
}

//double* BundleAdjuster::parameters;
double* BundleAdjuster::cam_param;
double* BundleAdjuster::point_param;
