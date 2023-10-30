# Create a library called "klib-inverse" (in Linux, this library is created
# with the name of either libklib-inverse.a or klib-inverse.so).
add_library (klib-inverse
  # list of cpp source files:
  src.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(klib-inverse PUBLIC
  # list of directories:
  .
  )
