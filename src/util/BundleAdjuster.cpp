#include "BundleAdjuster.h"
#include "snavely_reprojection_error.h"




BundleAdjuster::BundleAdjuster()
{

}

void BundleAdjuster::SetLinearSolver(Solver::Options* options)
{
  options->linear_solver_type = LinearSolverType::DENSE_SCHUR;
  options->preconditioner_type = PreconditionerType::JACOBI;
  options->visibility_clustering_type = VisibilityClusteringType::CANONICAL_VIEWS;
  options->sparse_linear_algebra_library_type = SparseLinearAlgebraLibraryType::SUITE_SPARSE;
  options->dense_linear_algebra_library_type = DenseLinearAlgebraLibraryType::EIGEN;
  options->use_explicit_schur_complement = false;
}

void BundleAdjuster::SetMinimizerOptions(Solver::Options* options)
{
  options->max_num_iterations = 10;
  options->minimizer_progress_to_stdout = true;
  options->num_threads = 4;
  options->eta = 1e-2;
  options->max_solver_time_in_seconds = 1e32;
  options->use_nonmonotonic_steps = false;

  options->minimizer_type = ceres::LINE_SEARCH;

  options->trust_region_strategy_type = LEVENBERG_MARQUARDT;
  options->dogleg_type = TRADITIONAL_DOGLEG;

  options->use_inner_iterations = false;
}

void BundleAdjuster::SetSolverOptions(Graph *graph, Solver::Options* options)
{
  SetMinimizerOptions(options);
  SetLinearSolver(options);
//  SetOrdering(graph, options);
}

void BundleAdjuster::SetOrdering(Graph *graph, Solver::Options* options)
{
    ceres::ParameterBlockOrdering* ordering =
        new ceres::ParameterBlockOrdering;
    //first eliminate points. after solvine cameras,substitute back to solve points
    // The points come before the cameras.
    PointPtrMap::iterator p_it = graph->getPoints().begin();
    for(; p_it != graph->getPoints().end();p_it++)
    {
        ordering->AddElementToGroup(p_it->second->getMutable(), 0);
    }
    FramePtrVector::iterator frame_it = graph->getFrames().begin();
    for (; frame_it != graph->getFrames().end(); frame_it++)
    {
      // When using axis-angle, there is a single parameter block for
      // the entire camera.
      ordering->AddElementToGroup((*frame_it)->getMutable(), 1);
    }

    options->linear_solver_ordering.reset(ordering);
}

void BundleAdjuster::BuildProblem(Graph *graph,Problem* problem)
{
    FramePtrVector pFrames = graph->getFrames();
    PointPtrMap pPoints = graph->getPoints();
    FramePtrVector::iterator pFrameIt = pFrames.begin();
    int camera_size = 9*pFrames.size();
    int point_size = 3*pPoints.size();
    int param_size = 9*pFrames.size()+3*pPoints.size();
    parameters = new double[param_size];
    double *param = new double[param_size];
    graph->getOptParameters(param);
    double* cam = parameters;
    double* point = parameters + camera_size;
    for(int i = 0; i < pFrames.size(); i++)
    {
        Frame::ObservationVector observations = pFrames[i]->getObservations();

        for(int j = 0; j < observations.size(); j++)
        {
            CostFunction* cost_function;
            cost_function = Error::Create(observations[j].second[0], observations[j].second[1]);
            LossFunction* loss_function = new HuberLoss(1.0);
//            double camera[9] = pFrames[i]->getMutable();
//            double point[3] = pPoints[observations[j].first]->getMutable();
            problem->AddResidualBlock(cost_function, loss_function, cam+i, point+observations[j].first);
        }
    }
}

void BundleAdjuster::solve(Graph *graph)
{
    Problem problem;
    Solver::Options options;
    BuildProblem(graph,&problem);
//    SetSolverOptions(graph,&options);
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
//    options.gradient_tolerance = 1e-16;
//    options.function_tolerance = 1e-16;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

double* BundleAdjuster::parameters;
