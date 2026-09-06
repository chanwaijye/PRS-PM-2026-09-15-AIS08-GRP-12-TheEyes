# runtime_agent -- the live end-to-end runner, record/replay, SQLite metrics.
#
# Links every other module. Everything except the librealsense capture path
# builds and tests with no hardware, no GPU and no ONNX Runtime, so the replay
# and mock paths are always available (AGENTS.md orchestration rule 4).
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
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
  src/runtime_config.cpp
  src/backends.cpp
  src/startup_checks.cpp
  src/pipeline.cpp
  src/viewer.cpp)

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

target_compile_options(the_eyes_runtime PRIVATE -Wall -Wextra -Wpedantic)

# The runner. Always built: replay, check and the mock backends need no
# hardware; `live` reports at runtime if librealsense was absent.
add_executable(the_eyes_run tools/the_eyes_run.cpp)
target_link_libraries(the_eyes_run PRIVATE the_eyes_runtime)
target_compile_options(the_eyes_run PRIVATE -Wall -Wextra -Wpedantic)

# Re-scores the occlusion ablation from a metrics log once ground truth exists.
add_executable(ablation_from_log tools/ablation_from_log.cpp)
target_link_libraries(ablation_from_log PRIVATE the_eyes_runtime)
target_compile_options(ablation_from_log PRIVATE -Wall -Wextra -Wpedantic)

if(THE_EYES_BUILD_TESTS)
  add_executable(test_runtime
    test/test_frame_pairer.cpp
    test/test_recorder_replay.cpp
    test/test_static_scene.cpp
    test/test_metrics_log.cpp
    test/test_runtime_config.cpp
    test/test_backends.cpp
    test/test_replay_pipeline.cpp)
  target_link_libraries(test_runtime
    PRIVATE the_eyes_runtime GTest::gtest GTest::gtest_main yaml-cpp SQLite::SQLite3)
  target_compile_options(test_runtime PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_runtime PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config"
    THE_EYES_REPO_ROOT="${CMAKE_SOURCE_DIR}/..")
  add_test(NAME runtime COMMAND test_runtime)
endif()
