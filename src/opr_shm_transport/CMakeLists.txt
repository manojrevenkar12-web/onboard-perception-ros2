cmake_minimum_required(VERSION 3.16)
project(opr_shm_transport)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(opr_msgs REQUIRED)
find_package(opr_shm REQUIRED)

add_library(opr_shm_transport SHARED src/image_publisher.cpp)
target_include_directories(opr_shm_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(opr_shm_transport PUBLIC
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${opr_msgs_TARGETS} opr::opr_shm)
target_compile_options(opr_shm_transport PRIVATE -Wall -Wextra -Wpedantic)

add_library(opr_shm_transport_plugins SHARED src/plugins.cpp)
target_link_libraries(opr_shm_transport_plugins PUBLIC opr_shm_transport image_transport::image_transport pluginlib::pluginlib)
target_compile_options(opr_shm_transport_plugins PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(image_transport plugins.xml)

install(TARGETS opr_shm_transport opr_shm_transport_plugins EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs image_transport opr_msgs opr_shm)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transport test/test_transport.cpp TIMEOUT 120)
  target_link_libraries(test_transport opr_shm_transport_plugins)
endif()

ament_package()
