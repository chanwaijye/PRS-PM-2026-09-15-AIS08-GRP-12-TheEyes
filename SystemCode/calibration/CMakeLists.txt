find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs highgui)

set(CALIB_SOURCES
  src/board.cpp
  src/observation.cpp
  src/solver.cpp
  src/synthetic.cpp
  src/report.cpp
  src/reregister.cpp)

# ChArUco detection needs opencv_contrib's aruco. The SOLVER does not -- it
# works on BoardObservations, which synthetic generation can produce -- so a
# machine without contrib can still build and fully test the calibration maths.
find_path(ARUCO_INCLUDE_DIR opencv2/aruco.hpp HINTS ${OpenCV_INCLUDE_DIRS})
find_library(ARUCO_LIBRARY NAMES opencv_aruco HINTS ${OpenCV_LIB_DIR} /usr/lib/x86_64-linux-gnu)

if(ARUCO_INCLUDE_DIR AND ARUCO_LIBRARY)
  message(STATUS "calibration: opencv_aruco found at ${ARUCO_LIBRARY}")
  list(APPEND CALIB_SOURCES src/charuco_detector.cpp src/marker_detector.cpp)
  set(THE_EYES_HAVE_ARUCO TRUE)
else()
  message(WARNING "calibration: opencv_aruco not found -- building without ChArUco "
                  "detection. The solver and its tests still build.")
  set(THE_EYES_HAVE_ARUCO FALSE)
endif()

add_library(the_eyes_calibration ${CALIB_SOURCES})

target_include_directories(the_eyes_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(the_eyes_calibration
  PUBLIC the_eyes_common ${OpenCV_LIBS}
  PRIVATE yaml-cpp)

if(THE_EYES_HAVE_ARUCO)
  target_link_libraries(the_eyes_calibration PUBLIC ${ARUCO_LIBRARY})
  target_compile_definitions(the_eyes_calibration PUBLIC THE_EYES_WITH_ARUCO=1)
endif()

if(THE_EYES_BUILD_TESTS)
  add_executable(test_calibration
    test/test_board.cpp
    test/test_synthetic.cpp
    test/test_solver.cpp
    test/test_report.cpp
    test/test_reregister.cpp)
  target_link_libraries(test_calibration
    PRIVATE the_eyes_calibration GTest::gtest GTest::gtest_main yaml-cpp)
  target_compile_definitions(test_calibration PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config")
  add_test(NAME calibration COMMAND test_calibration)
endif()

# calib_agent CLI. `mock` runs the whole pipeline on synthetic data with no
# cameras; `board` and `run` need opencv_aruco.
add_executable(calibrate tools/calibrate.cpp)
# Links fusion so `reregister` reads the previous extrinsics.yaml through the
# EXACT loader fusion uses. Reusing it rather than writing a second parser means
# a file this tool accepts is provably a file fusion will accept, and there is
# no second copy of the INTERFACES.md §4 format to drift. Tool-level only -- the
# calibration LIBRARY does not depend on fusion, so the layering still runs one
# way (calibration writes extrinsics, fusion consumes them).
target_link_libraries(calibrate
  PRIVATE the_eyes_calibration the_eyes_camera the_eyes_fusion ${OpenCV_LIBS})
