cmake_minimum_required(VERSION 3.16)
project(opr_sim VERSION 0.1.0 LANGUAGES CXX)

# Plain CMake, no ROS: the legacy program links this too.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(opr_sim src/scene.cpp)
target_include_directories(opr_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Bit-identical frames on x86 and aarch64: never fuse a*b+c into one rounding.
target_compile_options(opr_sim PUBLIC -ffp-contract=off PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wno-sign-conversion)
set_target_properties(opr_sim PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(opr_sim_dump tools/dump.cpp)
target_link_libraries(opr_sim_dump opr_sim)

install(TARGETS opr_sim opr_sim_dump EXPORT opr_simTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/opr_sim)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT opr_simTargets NAMESPACE opr:: DESTINATION share/opr_sim/cmake)
include(CMakePackageConfigHelpers)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/opr_simConfig.cmake
  "include(\${CMAKE_CURRENT_LIST_DIR}/cmake/opr_simTargets.cmake)\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/opr_simConfig.cmake DESTINATION share/opr_sim)
install(FILES package.xml DESTINATION share/opr_sim)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    add_executable(test_scene test/test_scene.cpp)
    target_link_libraries(test_scene opr_sim GTest::gtest_main)
    add_test(NAME test_scene COMMAND test_scene)
  endif()
endif()
