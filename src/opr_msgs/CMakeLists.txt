cmake_minimum_required(VERSION 3.16)
project(opr_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ShmImage.msg
  msg/FrameInfo.msg
  msg/Detection.msg
  msg/DetectionArray.msg
  msg/Track.msg
  msg/TrackArray.msg
  msg/Health.msg
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
