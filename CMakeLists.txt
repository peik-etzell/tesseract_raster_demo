cmake_minimum_required(VERSION 3.8)
project(tesseract_raster_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_rosutils REQUIRED)

# Planning server
add_library(${PROJECT_NAME}_planner src/planning_server.cpp)
ament_target_dependencies(${PROJECT_NAME}_planner Eigen3 tesseract_rosutils)
target_link_libraries(
  ${PROJECT_NAME}_planner
  tesseract::tesseract_common
  tesseract::tesseract_command_language
  tesseract::tesseract_environment
  tesseract::tesseract_task_composer
  tesseract::tesseract_task_composer_planning
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_motion_planners_trajopt
  console_bridge::console_bridge)
target_include_directories(
  ${PROJECT_NAME}_planner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# Demo node
add_executable(${PROJECT_NAME}_node src/demo.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp Eigen3)
target_link_libraries(
  ${PROJECT_NAME}_node ${PROJECT_NAME}_planner tesseract::tesseract_environment
  tesseract_monitoring::tesseract_monitoring_environment)
target_include_directories(
  ${PROJECT_NAME}_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
