# Find Eigen3 library.
# To install opencv on Ubuntu, do:
#    sudo apt install libeigen3-dev
find_package(Eigen3 REQUIRED)

# Any C++ source files needed to build this target (cpp-test).
add_executable(cpp-test
  # list of source cpp files:
  main.cpp
  test.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(cpp-test PUBLIC
  # list of include directories:
  # ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  )

# Any dependent libraires needed to build this target.
target_link_libraries(cpp-test PUBLIC
  # list of libraries:
  gtest
  klib-datatypes
  klib-forward
  klib-inverse
  klib-manipulator
  )

# Enable CMake’s test runner to discover the tests included in the
# binary, using the GoogleTest CMake module.
gtest_discover_tests(cpp-test)
