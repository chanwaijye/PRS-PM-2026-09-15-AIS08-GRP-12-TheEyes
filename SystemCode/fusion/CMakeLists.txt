add_library(the_eyes_fusion
  src/so3.cpp
  src/quaternion_average.cpp
  src/extrinsics.cpp
  src/config.cpp
  src/ekf.cpp
  src/symmetry.cpp
  src/fusion.cpp
  src/metrics.cpp)

target_include_directories(the_eyes_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(the_eyes_fusion
  PUBLIC the_eyes_common
  PRIVATE yaml-cpp)

target_compile_options(the_eyes_fusion PRIVATE -Wall -Wextra -Wpedantic)

if(THE_EYES_BUILD_TESTS)
  add_executable(test_fusion
    test/test_math.cpp
    test/test_extrinsics.cpp
    test/test_config.cpp
    test/test_ekf.cpp
    test/test_symmetry.cpp
    test/test_fusion.cpp
    test/test_ablation.cpp)
  target_link_libraries(test_fusion
    PRIVATE the_eyes_fusion GTest::gtest GTest::gtest_main yaml-cpp)
  target_compile_definitions(test_fusion PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config")
  add_test(NAME fusion COMMAND test_fusion)
endif()

# Demonstrates the ablation harness on a synthetic scene. Its output is clearly
# labelled synthetic -- it exercises the harness, it is not an evaluation result.
add_executable(fusion_ablation_demo tools/ablation_demo.cpp)
target_link_libraries(fusion_ablation_demo PRIVATE the_eyes_fusion)
