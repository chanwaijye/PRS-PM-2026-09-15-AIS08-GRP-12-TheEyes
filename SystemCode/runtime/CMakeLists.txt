# runtime_agent -- the live end-to-end runner, record/replay, SQLite metrics.
#
# Links every other module. Everything except the librealsense capture path
# builds and tests with no hardware, no GPU and no ONNX Runtime, so the replay
# and mock paths are always available (AGENTS.md orchestration rule 4).
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio)
find_package(SQLite3 REQUIRED)
find_package(Threads REQUIRED)

set(RUNTIME_SOURCES
  src/frame_pair.cpp
  src/frame_pairer.cpp
  src/session.cpp
  src/recorder.cpp
  src/replay_frame_source.cpp
  src/static_scene.cpp
  src/metrics_log.cpp
  src/marker_trays.cpp
  src/visibility.cpp
  src/depth_correction.cpp
  src/brightness.cpp
  src/view_evidence.cpp
  src/runtime_config.cpp
  src/backends.cpp
  src/fixed_box_detector.cpp
  src/startup_checks.cpp
  src/pipeline.cpp
  src/viewer.cpp
  src/console_panels.cpp
  src/console.cpp
  src/console_recorder.cpp)

if(THE_EYES_HAVE_REALSENSE)
  list(APPEND RUNTIME_SOURCES src/live_frame_source.cpp)
endif()

add_library(the_eyes_runtime ${RUNTIME_SOURCES})

target_include_directories(the_eyes_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# THE_EYES_WITH_REALSENSE / THE_EYES_WITH_ONNXRUNTIME arrive transitively as
# PUBLIC compile definitions of the camera and detector libraries.
target_link_libraries(the_eyes_runtime
  PUBLIC the_eyes_camera the_eyes_detector the_eyes_pose the_eyes_fusion
         the_eyes_scenegraph ${OpenCV_LIBS} Threads::Threads
  PRIVATE SQLite::SQLite3 yaml-cpp)

# The runner. Always built: replay, check and the mock backends need no
# hardware; `live` reports at runtime if librealsense was absent.
add_executable(the_eyes_run tools/the_eyes_run.cpp)
target_link_libraries(the_eyes_run PRIVATE the_eyes_runtime)

# Re-scores the occlusion ablation from a metrics log once ground truth exists.
add_executable(ablation_from_log tools/ablation_from_log.cpp)
target_link_libraries(ablation_from_log PRIVATE the_eyes_runtime)

# Fills a run's ground_truth table from the ArUco trays, so ablation_from_log
# has a yardstick. Links calibration for the marker detector -- tool level only,
# exactly as calibration's own `calibrate` links fusion, so the libraries keep
# their one-way layering.
add_executable(gt_from_trays tools/gt_from_trays.cpp)
target_link_libraries(gt_from_trays PRIVATE the_eyes_runtime the_eyes_calibration yaml-cpp)

# Measures how much of each object's ground-truth silhouette each camera sees,
# which is how the proposal defines the occlusion tiers (7.1).
add_executable(visibility_from_gt tools/visibility_from_gt.cpp)
target_link_libraries(visibility_from_gt PRIVATE the_eyes_runtime yaml-cpp)

# Fits each camera's depth scale against a recording of the bare table.
add_executable(depth_tare tools/depth_tare.cpp)
target_link_libraries(depth_tare PRIVATE the_eyes_runtime)

if(THE_EYES_BUILD_TESTS)
  add_executable(test_runtime
    test/test_frame_pairer.cpp
    test/test_recorder_replay.cpp
    test/test_static_scene.cpp
    test/test_metrics_log.cpp
    test/test_runtime_config.cpp
    test/test_backends.cpp
    test/test_fixed_box_detector.cpp
    test/test_replay_pipeline.cpp
    test/test_marker_trays.cpp
    test/test_visibility.cpp
    test/test_depth_correction.cpp
    test/test_brightness.cpp
    test/test_view_evidence.cpp
    test/test_gt_chain.cpp
    test/test_console_panels.cpp)
  # Links calibration so the ground-truth chain can be tested end to end
  # against a rendered marker. Test-only: the runtime LIBRARY does not depend
  # on calibration, only gt_from_trays does.
  target_link_libraries(test_runtime
    PRIVATE the_eyes_runtime the_eyes_calibration GTest::gtest GTest::gtest_main
            yaml-cpp SQLite::SQLite3)
  target_compile_definitions(test_runtime PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config"
    THE_EYES_REPO_ROOT="${CMAKE_SOURCE_DIR}/..")
  add_test(NAME runtime COMMAND test_runtime)
endif()
