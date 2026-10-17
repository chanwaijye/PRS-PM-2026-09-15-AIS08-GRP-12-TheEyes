add_library(the_eyes_scenegraph
  src/predicates.cpp
  src/pddl.cpp)

target_include_directories(the_eyes_scenegraph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(the_eyes_scenegraph
  PUBLIC the_eyes_common
  PRIVATE yaml-cpp)

if(THE_EYES_BUILD_TESTS)
  add_executable(test_scenegraph
    test/test_predicates.cpp
    test/test_pddl.cpp)
  target_link_libraries(test_scenegraph
    PRIVATE the_eyes_scenegraph GTest::gtest GTest::gtest_main)
  # Tolerances must come from config/predicates.yaml, never from constants in
  # the module -- the test suite asserts that by loading the real file.
  target_compile_definitions(test_scenegraph PRIVATE
    THE_EYES_CONFIG_DIR="${CMAKE_SOURCE_DIR}/../config")
  add_test(NAME scenegraph COMMAND test_scenegraph)
endif()
