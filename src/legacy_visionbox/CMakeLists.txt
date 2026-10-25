cmake_minimum_required(VERSION 3.16)
project(legacy_visionbox VERSION 2.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(opr_sim REQUIRED)
add_executable(visionbox visionbox.cpp)
# Same floating-point contract as the new pipeline, so the two can be compared bit for bit.
target_compile_options(visionbox PRIVATE -ffp-contract=off)
target_link_libraries(visionbox opr::opr_sim)

install(TARGETS visionbox RUNTIME DESTINATION lib/legacy_visionbox)
install(FILES visionbox.ini package.xml DESTINATION share/legacy_visionbox)
