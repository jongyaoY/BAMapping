#include "BundleAdjuster.h"
#include "snavely_reprojection_error.h"




BundleAdjuster::BundleAdjuster()
{

}

void BundleAdjuster::SetLinearSolver(Solver::Options* options)
{
  CHECK(StringToLinearSolverType(FLAGS_linear_solver,
                                 &options->linear_solver_type));
  CHECK(StringToPreconditionerType(FLAGS_preconditioner,
                                   &options->preconditioner_type));
  CHECK(StringToVisibilityClusteringType(FLAGS_visibility_clustering,
                                         &options->visibility_clustering_type));
  CHECK(StringToSparseLinearAlgebraLibraryType(
            FLAGS_sparse_linear_algebra_library,
            &options->sparse_linear_algebra_library_type));
  CHECK(StringToDenseLinearAlgebraLibraryType(
            FLAGS_dense_linear_algebra_library,
            &options->dense_linear_algebra_library_type));
  options->use_explicit_schur_complement = FLAGS_explicit_schur_complement;
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

void BundleAdjuster::SetSolverOptions(Graph *graph,
                               Solver::Options* options)
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
    Point *point = graph->getPoints()->data();
    for(int i = 0; i< graph->getPoints()->size(); i++)
    {
        ordering->AddElementToGroup(point[i].mutablePose(), 0);
    }
    Frame *frame = graph->getFrames()->data();
    for (int i = 0; i < graph->getFrames()->size(); i++)
    {
      // When using axis-angle, there is a single parameter block for
      // the entire camera.
      ordering->AddElementToGroup(frame[i].getMutable(), 1);
    }

    options->linear_solver_ordering.reset(ordering);
}


void BundleAdjuster::BuildProblem(Graph *graph,Problem* problem)
{
    FrameVector *pFrames = graph->getFrames();
    PointVector *pPoints = graph->getPoints();
    FrameVector::iterator frameIt = pFrames->begin();
    for(;frameIt!=pFrames->end();frameIt++)
    {

        Frame::ObservationVector observations = frameIt->getObservations();
        Frame::ObservationVector::const_iterator obsIt = observations.begin();
//        Point *point = &(*pPoints)[obsIt->first];
        Point *point = pPoints->data();
        for(;obsIt != observations.end(); obsIt++)
        {

            CostFunction* cost_function;
            cost_function = Error::Create(obsIt->second[0], obsIt->second[1]);
            LossFunction* loss_function = new HuberLoss(1.0);

            problem->AddResidualBlock(cost_function, loss_function, frameIt->getMutable(), (point + obsIt->first)->mutablePose());

        }
    }
}


void BundleAdjuster::solve(Graph *graph)
{
    Problem problem;
    BuildProblem(graph,&problem);
}
