#include <console_bridge/console.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/utils.h>  // toJointTrajectory
#include <tesseract_common/joint_state.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/timer.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>  // toToolPath
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_rosutils/plotting.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include <vector>
#include <tesseract_raster_demo/planning_server.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

static const std::string DEFAULT_PROFILE = "DEFAULT";

static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

static const int NUM_RASTERS = 8;

// Convert a 'pointing' vector to a corresponding Eigen::Quaterniond
Eigen::Quaterniond orientation(Eigen::Vector3d direction)
{
  direction.normalize();
  auto z = direction;
  auto y = Eigen::Vector3d::UnitZ().cross(z).normalized();
  auto x = y.cross(z);
  Eigen::Matrix3d res_mat;
  res_mat.col(0) = x;
  res_mat.col(1) = y;
  res_mat.col(2) = z;
  return Eigen::Quaterniond{ res_mat };
};

namespace planning_server
{

bool run(tesseract_environment::Environment::Ptr env, tesseract_common::ManipulatorInfo manipulator, bool debug = false)
{
  if (debug)
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

  tesseract_rosutils::ROSPlottingPtr plotter;
  plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(env->getSceneGraph()->getRoot());

  // Create Task Composer Plugin Factory
  const std::string share_dir(ament_index_cpp::get_package_share_directory("tesseract_task_composer"));
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  tesseract_planning::TaskComposerPluginFactory factory(config_path);

  auto program = [&] {
    tesseract_planning::CompositeInstruction program(
        DEFAULT_PROFILE, tesseract_planning::CompositeInstructionOrder::ORDERED, manipulator);

    // This changes!
    Eigen::Isometry3d target_tool_pose = Eigen::Translation3d(0.4, -0.3, 0.6) * orientation({ 1, 0, -1 });

    const auto joint_names = env->getActiveJointNames();
    const auto num_dof = (int)joint_names.size();

    const tesseract_planning::StateWaypointPoly zero_state_wp{ tesseract_planning::StateWaypoint(
        joint_names, Eigen::VectorXd::Zero(num_dof)) };

    // Start instruction
    {
      tesseract_planning::CompositeInstruction start_comp(DEFAULT_PROFILE);
      // start_comp.setDescription("Move to first raster");
      {
        tesseract_planning::MoveInstruction start_instr(
            zero_state_wp, tesseract_planning::MoveInstructionType::FREESPACE, DEFAULT_PROFILE);
        start_comp.appendMoveInstruction(start_instr);
      }
      // Move to first raster point
      {
        tesseract_planning::CartesianWaypointPoly fwp{ tesseract_planning::CartesianWaypoint(target_tool_pose) };
        tesseract_planning::MoveInstruction start_move_instr(fwp, tesseract_planning::MoveInstructionType::FREESPACE,
                                                             DEFAULT_PROFILE);
        start_comp.appendMoveInstruction(start_move_instr);
      }
      program.push_back(start_comp);
    }

    // Rasters
    for (int raster = 0; raster < NUM_RASTERS; ++raster)
    {
      tesseract_planning::CompositeInstruction raster_comp(DEFAULT_PROFILE);
      for (int raster_wp = 0; raster_wp < 6; ++raster_wp)
      {
        target_tool_pose = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) * target_tool_pose;
        tesseract_planning::CartesianWaypointPoly rwp = tesseract_planning::CartesianWaypoint(target_tool_pose);
        raster_comp.appendMoveInstruction(
            tesseract_planning::MoveInstruction(rwp, tesseract_planning::MoveInstructionType::LINEAR, DEFAULT_PROFILE));
      }
      program.push_back(raster_comp);

      // Transitions
      if (raster != NUM_RASTERS - 1)
      {
        target_tool_pose = Eigen::Translation3d(0, 0, 0.05) * target_tool_pose;
        tesseract_planning::CartesianWaypointPoly next_raster_start =
            tesseract_planning::CartesianWaypoint(target_tool_pose);

        tesseract_planning::CompositeInstruction trans_comp(DEFAULT_PROFILE);
        trans_comp.appendMoveInstruction(tesseract_planning::MoveInstruction(
            next_raster_start, tesseract_planning::MoveInstructionType::FREESPACE, DEFAULT_PROFILE));
        program.push_back(trans_comp);
      }
    }

    // End instruction
    {
      tesseract_planning::CompositeInstruction end_comp(DEFAULT_PROFILE);
      {
        tesseract_planning::MoveInstruction end_move_instr(
            zero_state_wp, tesseract_planning::MoveInstructionType::FREESPACE, DEFAULT_PROFILE);
        end_comp.appendMoveInstruction(end_move_instr);
      }
      program.push_back(end_comp);
    }
    return program;
  }();

  program.print("Program: ");

  tesseract_common::Toolpath program_toolpath = tesseract_planning::toToolpath(program, *env);
  plotter->plotMarker(tesseract_visualization::ToolpathMarker(program_toolpath));

  auto planner_profiles = std::make_shared<tesseract_planning::ProfileDictionary>();

  // DESCARTES
  {
    auto descartes_profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<float>>();
    descartes_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
      return tesseract_planning::sampleToolZAxis(tool_pose, 0.05);
    };
    descartes_profile->debug = debug;
    descartes_profile->use_redundant_joint_solutions = true;
    descartes_profile->num_threads = debug ? 1 : static_cast<int>(std::thread::hardware_concurrency());
    planner_profiles->addProfile<tesseract_planning::DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE,
                                                                                  DEFAULT_PROFILE, descartes_profile);
  }

  // TRAJOPT
  {
    {
      auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
      // Underconstrained orientation
      trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Ones(6);
      trajopt_plan_profile->cartesian_coeff(3) = 0;
      trajopt_plan_profile->cartesian_coeff(4) = 0;
      trajopt_plan_profile->cartesian_coeff(5) = 0;
      // trajopt_plan_profile->term_type = trajopt::TermType::TT_COST;
      planner_profiles->addProfile<tesseract_planning::TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE,
                                                                           trajopt_plan_profile);
    }

    {
      auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
      trajopt_composite_profile->collision_constraint_config.enabled = false;
      trajopt_composite_profile->collision_cost_config.enabled = true;
      trajopt_composite_profile->collision_cost_config.safety_margin = 0.025;
      trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
      trajopt_composite_profile->collision_cost_config.coeff = 20;
      trajopt_composite_profile->smooth_velocities = true;
      trajopt_composite_profile->smooth_accelerations = true;
      trajopt_composite_profile->smooth_jerks = true;
      planner_profiles->addProfile<tesseract_planning::TrajOptCompositeProfile>(
          TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE, trajopt_composite_profile);
    }

    {
      auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
      // trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
      trajopt_solver_profile->opt_info.num_threads = 0;
      trajopt_solver_profile->opt_info.max_iter = 200;
      // trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
      // trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
      planner_profiles->addProfile<tesseract_planning::TrajOptSolverProfile>(TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE,
                                                                             trajopt_solver_profile);
    }
  }

  // Taskflow executor
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
  tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode("RasterFtPipeline");
  const std::string input_key = task->getInputKeys().front();
  const std::string output_key = task->getOutputKeys().front();

  // Create Task Input Data
  tesseract_planning::TaskComposerDataStorage input_data;
  input_data.setData(input_key, program);

  // Create Task Composer Problem
  auto problem = std::make_unique<tesseract_planning::PlanningTaskComposerProblem>(env, input_data, planner_profiles);

  tesseract_planning::TaskComposerInput input(std::move(problem));
  // Solve problem
  {
    tesseract_common::Timer stopwatch;
    stopwatch.start();
    input.dotgraph = true;
    tesseract_planning::TaskComposerFuture::UPtr future = executor->run(*task, input);
    future->wait();
    stopwatch.stop();
  }

  // Save dot graph
  {
    std::ofstream tc_out_data;
    tc_out_data.open("task_composer_graph.dot");
    task->dump(tc_out_data, nullptr, input.task_infos.getInfoMap());
    tc_out_data.close();
  }

  plotter->waitForConnection(30);
  auto ci = input.data_storage.getData(output_key).as<tesseract_planning::CompositeInstruction>();
  tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(ci, *env);
  tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(ci);
  auto state_solver = env->getStateSolver();
  plotter->plotMarker(tesseract_visualization::ToolpathMarker(toolpath));
  plotter->plotTrajectory(trajectory, *state_solver);

  return input.isSuccessful();
}
}  // namespace planning_server
