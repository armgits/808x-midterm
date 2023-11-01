# To find Eigen3 package and enable matrix multiplications
find_package(Eigen3 REQUIRED)


# Create a library called "klib-forward" (in Linux, this library is created
# with the name of either libForward.a or klib-forward.so).
add_library (klib-forward
  # list of cpp source files:
  src.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(klib-forward PUBLIC
  # list of directories:
  .
  ${EIGEN3_INCLUDE_DIRS}
  )

target_link_libraries(klib-forward PUBLIC
  klib-datatypes
  klib-manipulator
  )


