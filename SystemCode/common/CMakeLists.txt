# Header-only contract library. Everything downstream compiles against this so
# that no module declares its own copy of ObjectState.
add_library(the_eyes_common INTERFACE)
target_include_directories(the_eyes_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(the_eyes_common INTERFACE Eigen3::Eigen)
target_compile_features(the_eyes_common INTERFACE cxx_std_17)
