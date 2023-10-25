# Create a library called "Inverse" (in Linux, this library is created
# with the name of either libInverse.a or Inverse.so).
add_library (Inverse
  # list of cpp source files:
  src.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(Inverse PUBLIC
  # list of directories:
  .
  )
