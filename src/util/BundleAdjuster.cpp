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

void BundleAdjuster::SetOrdering(BALProblem* bal_problem, Solver::Options* options)
{
  const int num_points = bal_problem->num_points();
  const int point_block_size = bal_problem->point_block_size();
  double* points = bal_problem->mutable_points();

  const int num_cameras = bal_problem->num_cameras();
  const int camera_block_size = bal_problem->camera_block_size();
  double* cameras = bal_problem->mutable_cameras();

  if (options->use_inner_iterations)
  {


  }

  // Bundle adjustment problems have a sparsity structure that makes
  // them amenable to more specialized and much more efficient
  // solution strategies. The SPARSE_SCHUR, DENSE_SCHUR and
  // ITERATIVE_SCHUR solvers make use of this specialized
  // structure.
  //
  // This can either be done by specifying Options::ordering_type =
  // ceres::SCHUR, in which case Ceres will automatically determine
  // the right ParameterBlock ordering, or by manually specifying a
  // suitable ordering vector and defining
  // Options::num_eliminate_blocks.

  ceres::ParameterBlockOrdering* ordering =
      new ceres::ParameterBlockOrdering;
  //first eliminate points. after solvine cameras,substitute back to solve points
  // The points come before the cameras.
  for (int i = 0; i < num_points; ++i)
  {
    ordering->AddElementToGroup(points + point_block_size * i, 0);
  }

  for (int i = 0; i < num_cameras; ++i)
  {
    // When using axis-angle, there is a single parameter block for
    // the entire camera.
    ordering->AddElementToGroup(cameras + camera_block_size * i, 1);
  }

  options->linear_solver_ordering.reset(ordering);
}

void BundleAdjuster::SetMinimizerOptions(Solver::Options* options)
{
  options->max_num_iterations = FLAGS_num_iterations;
  options->minimizer_progress_to_stdout = true;
  options->num_threads = FLAGS_num_threads;
  options->eta = FLAGS_eta;
  options->max_solver_time_in_seconds = FLAGS_max_solver_time;
  options->use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
  if (FLAGS_line_search) {
    options->minimizer_type = ceres::LINE_SEARCH;
  }

  CHECK(StringToTrustRegionStrategyType(FLAGS_trust_region_strategy,
                                        &options->trust_region_strategy_type));
  CHECK(StringToDoglegType(FLAGS_dogleg, &options->dogleg_type));
  options->use_inner_iterations = FLAGS_inner_iterations;
}

void BundleAdjuster::SetSolverOptionsFromFlags(BALProblem* bal_problem,
                               Solver::Options* options)
{
  SetMinimizerOptions(options);
  SetLinearSolver(options);
  SetOrdering(bal_problem, options);
}

void BundleAdjuster::BuildProblem(BALProblem* bal_problem, Problem* problem)
{
  const int point_block_size = bal_problem->point_block_size();
  const int camera_block_size = bal_problem->camera_block_size();
  double* points = bal_problem->mutable_points();
  double* cameras = bal_problem->mutable_cameras();

  // Observations is 2*num_observations long array observations =
  // [u_1, u_2, ... , u_n], where each u_i is two dimensional, the x
  // and y positions of the observation.
  const double* observations = bal_problem->observations();
  for (int i = 0; i < bal_problem->num_observations(); ++i)
  {
    CostFunction* cost_function;
    // Each Residual block takes a point and a camera as input and
    // outputs a 2 dimensional residual.
    cost_function =
        (FLAGS_use_quaternions)
        ? SnavelyReprojectionErrorWithQuaternions::Create(
            observations[2 * i + 0],
            observations[2 * i + 1])
        : SnavelyReprojectionError::Create(
            observations[2 * i + 0],
            observations[2 * i + 1]);

    // If enabled use Huber's loss function.
    LossFunction* loss_function = FLAGS_robustify ? new HuberLoss(1.0) : NULL;

    // Each observation correponds to a pair of a camera and a point
    // which are identified by camera_index()[i] and point_index()[i]
    // respectively.
    double* camera =
        cameras + camera_block_size * bal_problem->camera_index()[i];
    double* point = points + point_block_size * bal_problem->point_index()[i];
    problem->AddResidualBlock(cost_function, loss_function, camera, point);
  }

  if (FLAGS_use_quaternions && FLAGS_use_local_parameterization)
  {
    LocalParameterization* camera_parameterization =
        new ProductParameterization(
            new QuaternionParameterization(),
            new IdentityParameterization(6));
    for (int i = 0; i < bal_problem->num_cameras(); ++i)
    {
      problem->SetParameterization(cameras + camera_block_size * i,
                                   camera_parameterization);
    }
  }
}

void BundleAdjuster::SolveProblem(const char* filename)
{
  BALProblem bal_problem(filename, FLAGS_use_quaternions);

  if (!FLAGS_initial_ply.empty())
  {
    bal_problem.WriteToPLYFile(FLAGS_initial_ply);
  }

  Problem problem;

  srand(FLAGS_random_seed);
  bal_problem.Normalize();
  bal_problem.Perturb(FLAGS_rotation_sigma,
                      FLAGS_translation_sigma,
                      FLAGS_point_sigma);

  BuildProblem(&bal_problem, &problem);
  Solver::Options options;
  SetSolverOptionsFromFlags(&bal_problem, &options);
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  if (!FLAGS_final_ply.empty()) {
    bal_problem.WriteToPLYFile(FLAGS_final_ply);
  }
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
        for(;obsIt != observations.end(); obsIt++)
        {
            Point point = (*pPoints)[obsIt->first];
            CostFunction* cost_function;
            cost_function = Error::Create(obsIt->second[0], obsIt->second[1]);
            LossFunction* loss_function = new HuberLoss(1.0);

            problem->AddResidualBlock(cost_function, loss_function, frameIt->getMutable(), point.mutablePose());
        }
    }
}
