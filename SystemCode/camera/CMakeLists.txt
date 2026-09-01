find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

set(CAMERA_SOURCES
  src/config.cpp
  src/frame.cpp
  src/sync.cpp
  src/depth_noise.cpp
  src/device_registry.cpp)

# librealsense is optional. Everything that can be reasoned about without a
# camera -- config validation, timestamp-skew analysis, plane fitting, depth
# back-projection -- is pure C++ and always built and tested. Only the capture
# wrapper needs the SDK.
find_path(REALSENSE_INCLUDE_DIR librealsense2/rs.hpp HINTS /usr/local/include /usr/include)
find_library(REALSENSE_LIBRARY NAMES realsense2 HINTS /usr/local/lib /usr/lib
             /usr/lib/x86_64-linux-gnu)

if(REALSENSE_INCLUDE_DIR AND REALSENSE_LIBRARY)
  message(STATUS "camera: librealsense found at ${REALSENSE_LIBRARY}")
  list(APPEND CAMERA_SOURCES src/realsense_camera.cpp)
  set(THE_EYES_HAVE_REALSENSE TRUE)
else()
  message(WARNING "camera: librealsense not found -- building without capture. "
                  "Config, sync analysis, and depth-noise maths still build and test.")
  set(THE_EYES_HAVE_REALSENSE FALSE)
endif()

add_library(the_eyes_camera ${CAMERA_SOURCES})

target_include_directories(the_eyes_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(the_eyes_camera
  PUBLIC the_eyes_common ${OpenCV_LIBS}
  PRIVATE yaml-cpp)

target_compile_options(the_eyes_camera PRIVATE -Wall -Wextra -Wpedantic)

if(THE_EYES_HAVE_REALSENSE)
  target_include_directories(the_eyes_camera PUBLIC ${REALSENSE_INCLUDE_DIR})
  target_link_libraries(the_eyes_camera PUBLIC ${REALSENSE_LIBRARY})
  target_compile_definitions(the_eyes_camera PUBLIC THE_EYES_WITH_REALSENSE=1)
endif()

if(THE_EYES_BUILD_TESTS)
  add_executable(test_camera
    test/test_config.cpp
    test/test_sync.cpp
    test/test_depth_noise.cpp)
  target_link_libraries(test_camera
    PRIVATE the_eyes_camera GTest::gtest GTest::gtest_main yaml-cpp)
  target_compile_definitions(test_camera PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config")
  add_test(NAME camera COMMAND test_camera)
endif()

if(THE_EYES_HAVE_REALSENSE)
  add_executable(camera_probe tools/camera_probe.cpp)
  target_link_libraries(camera_probe PRIVATE the_eyes_camera)

  add_executable(depth_characterise tools/depth_characterise.cpp)
  target_link_libraries(depth_characterise PRIVATE the_eyes_camera)

  # Backs up the per-unit factory calibration, which is not reproducible
  # without Intel's rig.
  add_executable(backup_calibration tools/backup_calibration.cpp)
  target_link_libraries(backup_calibration PRIVATE the_eyes_camera ${REALSENSE_LIBRARY})
endif()
