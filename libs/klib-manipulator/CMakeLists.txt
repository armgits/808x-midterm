# Create a library called "klib-manipulator" (in Linux, this library is created
# with the name of either klib-manipulator.a or Manipulator.so).
add_library (klib-manipulator
  # list of cpp source files:
  src.cpp
  )

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(klib-manipulator PUBLIC
  # list of directories:
  .
  )

target_link_libraries(klib-manipulator PUBLIC
  klib-datatypes
  )