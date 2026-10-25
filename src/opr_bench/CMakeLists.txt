cmake_minimum_required(VERSION 3.16)
project(opr_bench)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(opr_msgs REQUIRED)
find_package(opr_shm REQUIRED)
find_package(opr_shm_transport REQUIRED)

add_library(opr_bench_nodes SHARED src/bench_nodes.cpp)
target_link_libraries(opr_bench_nodes PUBLIC rclcpp::rclcpp rclcpp_components::component
  image_transport::image_transport ${opr_msgs_TARGETS} opr::opr_shm opr_shm_transport::opr_shm_transport)
rclcpp_components_register_node(opr_bench_nodes PLUGIN "opr::bench::BenchPublisher" EXECUTABLE bench_pub)
rclcpp_components_register_node(opr_bench_nodes PLUGIN "opr::bench::BenchSubscriber" EXECUTABLE bench_sub)

add_executable(bench_intra src/bench_intra.cpp)
target_link_libraries(bench_intra opr_bench_nodes)
install(TARGETS bench_intra RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS opr_bench_nodes ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(PROGRAMS scripts/run_transport.py scripts/run_pipeline.py DESTINATION lib/${PROJECT_NAME})
ament_package()
