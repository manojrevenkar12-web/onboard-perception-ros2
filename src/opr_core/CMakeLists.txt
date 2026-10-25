cmake_minimum_required(VERSION 3.16)
project(opr_core VERSION 0.1.0 LANGUAGES CXX)

# Plain C++17, no ROS: the same library runs inside the ROS 2 nodes, in the single-process
# fallback and in the equivalence tests against visionbox 2.3.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(opr_core
  src/image.cpp
  src/rectify.cpp
  src/detect.cpp
  src/assign.cpp
  src/track.cpp
  src/telemetry.cpp
  src/pipeline.cpp)
target_include_directories(opr_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Never fuse a*b+c: results must match visionbox 2.3 and be the same on x86 and aarch64.
target_compile_options(opr_core PUBLIC -ffp-contract=off
  PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
set_target_properties(opr_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(opr_sim REQUIRED)
add_executable(opr_replay tools/replay.cpp)
target_link_libraries(opr_replay opr_core opr::opr_sim)
add_executable(opr_evaluate tools/evaluate.cpp)
target_link_libraries(opr_evaluate opr_core opr::opr_sim)

install(TARGETS opr_core EXPORT opr_coreTargets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS opr_replay opr_evaluate RUNTIME DESTINATION lib/opr_core)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT opr_coreTargets NAMESPACE opr:: DESTINATION share/opr_core/cmake)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/opr_coreConfig.cmake
  "include(\${CMAKE_CURRENT_LIST_DIR}/cmake/opr_coreTargets.cmake)\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/opr_coreConfig.cmake package.xml DESTINATION share/opr_core)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  foreach(t telemetry rectify detect track assign equivalence)
    add_executable(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} opr_core opr::opr_sim GTest::gtest_main)
    target_compile_definitions(test_${t} PRIVATE OPR_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden")
    add_test(NAME test_${t} COMMAND test_${t})
  endforeach()
endif()
