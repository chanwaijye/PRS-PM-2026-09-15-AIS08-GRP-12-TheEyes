find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(the_eyes_pose
  src/mesh_registry.cpp
  src/roi.cpp
  src/pose_estimator.cpp
  src/mock_estimator.cpp
  src/pose_agent.cpp
  src/foundationpose_client.cpp)

target_include_directories(the_eyes_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(the_eyes_pose
  PUBLIC the_eyes_common ${OpenCV_LIBS}
  PRIVATE yaml-cpp)

target_compile_options(the_eyes_pose PRIVATE -Wall -Wextra -Wpedantic)

if(THE_EYES_BUILD_TESTS)
  add_executable(test_pose
    test/test_mesh_registry.cpp
    test/test_roi.cpp
    test/test_pose_agent.cpp
    test/test_foundationpose_client.cpp)
  target_link_libraries(test_pose
    PRIVATE the_eyes_pose GTest::gtest GTest::gtest_main)
  target_compile_definitions(test_pose PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config")
  add_test(NAME pose COMMAND test_pose)
endif()

# End-to-end probe against the real FoundationPose server. Not a unit test --
# it needs the Python server, CUDA, and the demo data. See the module README.
add_executable(foundationpose_probe tools/foundationpose_probe.cpp)
target_link_libraries(foundationpose_probe PRIVATE the_eyes_pose)
