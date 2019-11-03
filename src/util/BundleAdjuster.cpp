#include "BundleAdjuster.h"
#include "snavely_reprojection_error.h"




BundleAdjuster::BundleAdjuster()
{

}

void BundleAdjuster::SetLinearSolver(Solver::Options* options)
{
  options->linear_solver_type = LinearSolverType::SPARSE_SCHUR;
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
  SetOrdering(graph, options);
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
    PointPtrMap::iterator pPointIt = pPoints.begin();
    for(; pFrameIt!=pFrames.end(); pFrameIt++)
    {

        Frame::ObservationVector observations = (*pFrameIt)->getObservations();
        Frame::ObservationVector::const_iterator obsIt = observations.begin();
        for(;obsIt != observations.end(); obsIt++)
        {

            CostFunction* cost_function;
            cost_function = Error::Create(obsIt->second[0], obsIt->second[1]);
            LossFunction* loss_function = new HuberLoss(1.0);

            problem->AddResidualBlock(cost_function, loss_function, (*pFrameIt)->getMutable(), pPoints[obsIt->first]->getMutable());

        }
    }
}

void BundleAdjuster::solve(Graph *graph)
{
    Problem problem;
    Solver::Options options;
    BuildProblem(graph,&problem);
    SetSolverOptions(graph,&options);
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}
