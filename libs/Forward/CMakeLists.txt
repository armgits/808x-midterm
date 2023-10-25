# Create a library called "Forward" (in Linux, this library is created
# with the name of either libForward.a or Forward.so).
add_library (Forward
  # list of cpp source files:
  src.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(Forward PUBLIC
  # list of directories:
  .
  )
