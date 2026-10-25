cmake_minimum_required(VERSION 3.16)
project(opr_v4l2 VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(opr_sim REQUIRED)
find_package(Threads REQUIRED)

add_library(opr_v4l2 src/camera.cpp src/syscalls.cpp src/fake_device.cpp)
target_include_directories(opr_v4l2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(opr_v4l2 PUBLIC opr::opr_sim Threads::Threads)
target_compile_options(opr_v4l2 PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
set_target_properties(opr_v4l2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(opr_v4l2_probe tools/probe.cpp)
target_link_libraries(opr_v4l2_probe opr_v4l2)

install(TARGETS opr_v4l2 EXPORT opr_v4l2Targets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS opr_v4l2_probe RUNTIME DESTINATION lib/opr_v4l2)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT opr_v4l2Targets NAMESPACE opr:: DESTINATION share/opr_v4l2/cmake)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/opr_v4l2Config.cmake
  "include(CMakeFindDependencyMacro)\nfind_dependency(opr_sim)\nfind_dependency(Threads)\ninclude(\${CMAKE_CURRENT_LIST_DIR}/cmake/opr_v4l2Targets.cmake)\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/opr_v4l2Config.cmake package.xml DESTINATION share/opr_v4l2)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_camera test/test_camera.cpp)
  target_link_libraries(test_camera opr_v4l2 GTest::gtest_main)
  add_test(NAME test_camera COMMAND test_camera)
endif()
