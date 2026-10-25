cmake_minimum_required(VERSION 3.16)
project(opr_shm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(opr_shm src/segment.cpp)
target_include_directories(opr_shm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(opr_shm PUBLIC rt)
target_compile_options(opr_shm PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
set_target_properties(opr_shm PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS opr_shm EXPORT opr_shmTargets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT opr_shmTargets NAMESPACE opr:: DESTINATION share/opr_shm/cmake)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/opr_shmConfig.cmake
  "include(\${CMAKE_CURRENT_LIST_DIR}/cmake/opr_shmTargets.cmake)\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/opr_shmConfig.cmake package.xml DESTINATION share/opr_shm)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_segment test/test_segment.cpp)
  target_link_libraries(test_segment opr_shm GTest::gtest_main)
  add_test(NAME test_segment COMMAND test_segment)
endif()
