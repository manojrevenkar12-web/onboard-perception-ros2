cmake_minimum_required(VERSION 3.16)
project(opr_pipeline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(image_transport REQUIRED)
find_package(opr_msgs REQUIRED)
find_package(opr_core REQUIRED)
find_package(opr_v4l2 REQUIRED)
find_package(opr_shm REQUIRED)
find_package(opr_shm_transport REQUIRED)

add_library(opr_pipeline_components SHARED
  src/camera_node.cpp
  src/rectify_node.cpp
  src/detector_node.cpp
  src/tracker_node.cpp
  src/telemetry_bridge_node.cpp
  src/health_node.cpp)
target_include_directories(opr_pipeline_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(opr_pipeline_components PUBLIC
  rclcpp::rclcpp rclcpp_components::component rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS} ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater image_transport::image_transport
  ${opr_msgs_TARGETS} opr::opr_core opr::opr_v4l2 opr::opr_shm opr_shm_transport::opr_shm_transport)
target_compile_options(opr_pipeline_components PRIVATE -Wall -Wextra -Wpedantic)

rclcpp_components_register_node(opr_pipeline_components PLUGIN "opr::pipeline::CameraNode" EXECUTABLE camera_node)
rclcpp_components_register_node(opr_pipeline_components PLUGIN "opr::pipeline::RectifyNode" EXECUTABLE rectify_node)
rclcpp_components_register_node(opr_pipeline_components PLUGIN "opr::pipeline::DetectorNode" EXECUTABLE detector_node)
rclcpp_components_register_node(opr_pipeline_components PLUGIN "opr::pipeline::TrackerNode" EXECUTABLE tracker_node)
rclcpp_components_register_node(opr_pipeline_components PLUGIN "opr::pipeline::TelemetryBridgeNode" EXECUTABLE telemetry_bridge_node)
rclcpp_components_register_node(opr_pipeline_components PLUGIN "opr::pipeline::HealthNode" EXECUTABLE health_node)

install(TARGETS opr_pipeline_components EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_joiner test/test_joiner.cpp)
  target_link_libraries(test_joiner opr_pipeline_components)
  ament_add_gtest(test_camera_node test/test_camera_node.cpp TIMEOUT 120)
  target_link_libraries(test_camera_node opr_pipeline_components)

  set(golden_dir ${CMAKE_CURRENT_SOURCE_DIR}/../opr_core/test/golden)
  add_launch_test(test/test_equivalence_composed.py TIMEOUT 300 ENV OPR_GOLDEN_DIR=${golden_dir})
  add_launch_test(test/test_equivalence_split.py TIMEOUT 300 ENV OPR_GOLDEN_DIR=${golden_dir})
  add_launch_test(test/test_camera_unplug.py TIMEOUT 120)
endif()

ament_package()
