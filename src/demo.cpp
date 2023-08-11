#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <thread>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_demo/planning_server.h>

// To use existing example.rviz config
static const std::string MONITOR_NAMESPACE = "tesseract_ros_examples";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("demo_application");

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("tesseract_demo_node");
  std::thread spinner{ [node]() { rclcpp::spin(node); } };

  // ROS parameters
  node->declare_parameter("robot_description", "");
  node->declare_parameter("robot_description_semantic", "");
  bool debug = node->declare_parameter("debug", false);

  // Create monitor
  auto monitor =
      std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node, "robot_description", MONITOR_NAMESPACE);
  monitor->startPublishingEnvironment();

  auto env = monitor->getEnvironment();

  tesseract_common::ManipulatorInfo manip("manipulator", "base_link", "tool0");

  planning_server::run(env, manip, debug);

  spinner.join();

  return 0;
}
