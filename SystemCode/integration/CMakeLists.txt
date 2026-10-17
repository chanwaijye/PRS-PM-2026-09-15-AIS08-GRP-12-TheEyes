# Pipeline-level tests spanning module boundaries. Owned at the integration
# level because CODING_AGENTS.md forbids one module's agent editing another's.
if(THE_EYES_BUILD_TESTS)
  add_executable(test_pipeline
    test/test_pipeline.cpp
    test/test_full_pipeline.cpp
    test/test_calibration_contract.cpp)
  target_link_libraries(test_pipeline
    PRIVATE the_eyes_fusion the_eyes_scenegraph the_eyes_pose the_eyes_calibration
            GTest::gtest GTest::gtest_main)
  add_test(NAME pipeline COMMAND test_pipeline)
endif()

# The whole chain end to end on synthetic input. Its output is labelled
# synthetic -- it demonstrates the contract chain, it is not an evaluation.
add_executable(pipeline_demo tools/pipeline_demo.cpp)
target_link_libraries(pipeline_demo
  PRIVATE the_eyes_fusion the_eyes_scenegraph the_eyes_pose)
