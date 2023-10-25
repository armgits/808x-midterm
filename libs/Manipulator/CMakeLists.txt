# Create a library called "Manipulator" (in Linux, this library is created
# with the name of either libManipulator.a or Manipulator.so).
add_library (Manipulator
  # list of cpp source files:
  src.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(Manipulator PUBLIC
  # list of directories:
  .
  )
