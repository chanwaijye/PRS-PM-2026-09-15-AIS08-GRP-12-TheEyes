cmake_minimum_required(VERSION 3.22)
project(the_eyes LANGUAGES CXX)

# CLAUDE.md: C++17 for all runtime code.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(THE_EYES_BUILD_TESTS "Build unit tests" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

if(THE_EYES_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
endif()

add_subdirectory(common)
add_subdirectory(scenegraph)
add_subdirectory(fusion)
add_subdirectory(camera)
add_subdirectory(calibration)
add_subdirectory(detector)
add_subdirectory(pose)
add_subdirectory(integration)
