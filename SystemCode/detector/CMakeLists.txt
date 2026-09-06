find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ONNX Runtime is optional. The valuable, failure-prone logic here -- letterbox
# inversion, YOLOv8 decode, NMS, tracking -- is pure C++ and is always built and
# tested. Only the inference binding needs the runtime, so a machine without it
# can still build and test the module.
set(THE_EYES_ONNXRUNTIME_ROOT "$ENV{HOME}/opt/onnxruntime-linux-x64-1.29.0"
    CACHE PATH "Root of an ONNX Runtime C++ distribution")

find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${THE_EYES_ONNXRUNTIME_ROOT}/include" /usr/include /usr/local/include)
find_library(ORT_LIBRARY NAMES onnxruntime
  HINTS "${THE_EYES_ONNXRUNTIME_ROOT}/lib" /usr/lib /usr/local/lib)

set(DETECTOR_SOURCES
  src/detector.cpp
  src/depth_encoding.cpp
  src/letterbox.cpp
  src/postprocess.cpp
  src/tracker.cpp
  src/mock_detector.cpp)

if(ORT_INCLUDE_DIR AND ORT_LIBRARY)
  message(STATUS "detector: ONNX Runtime found at ${ORT_LIBRARY}")
  list(APPEND DETECTOR_SOURCES src/onnx_detector.cpp)
  set(THE_EYES_HAVE_ORT TRUE)
  set(THE_EYES_HAVE_ORT TRUE PARENT_SCOPE)
else()
  message(WARNING "detector: ONNX Runtime not found -- building without the "
                  "inference backend. Set THE_EYES_ONNXRUNTIME_ROOT to enable it.")
  set(THE_EYES_HAVE_ORT FALSE)
  set(THE_EYES_HAVE_ORT FALSE PARENT_SCOPE)
endif()

add_library(the_eyes_detector ${DETECTOR_SOURCES})

target_include_directories(the_eyes_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(the_eyes_detector
  PUBLIC the_eyes_common ${OpenCV_LIBS}
  PRIVATE yaml-cpp)

target_compile_options(the_eyes_detector PRIVATE -Wall -Wextra -Wpedantic)

if(THE_EYES_HAVE_ORT)
  target_include_directories(the_eyes_detector PUBLIC ${ORT_INCLUDE_DIR})
  target_link_libraries(the_eyes_detector PUBLIC ${ORT_LIBRARY})
  target_compile_definitions(the_eyes_detector PUBLIC THE_EYES_WITH_ONNXRUNTIME=1)
endif()

if(THE_EYES_BUILD_TESTS)
  add_executable(test_detector
    test/test_depth_encoding.cpp
    test/test_letterbox.cpp
    test/test_postprocess.cpp
    test/test_tracker.cpp
    test/test_mock_detector.cpp)
  target_link_libraries(test_detector
    PRIVATE the_eyes_detector GTest::gtest GTest::gtest_main yaml-cpp)
  target_compile_definitions(test_detector PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config")
  add_test(NAME detector COMMAND test_detector)
endif()

if(THE_EYES_HAVE_ORT)
  # Parity harness: runs the C++ inference path on one image and dumps raw
  # detections, so the Python reference can be diffed against it.
  add_executable(detector_parity tools/detector_parity.cpp)
  target_link_libraries(detector_parity PRIVATE the_eyes_detector)
endif()
