# Any C++ source files needed to build this target (klib-demo).
add_executable(klib-demo
  # list of source cpp files:
  main.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(klib-demo PUBLIC
  # list inclue directories:
  # ${CMAKE_SOURCE_DIR}/include
)

# Any dependent libraires needed to build this target.
target_link_libraries(klib-demo PUBLIC
  # list of libraries
  klib-datatypes
  klib-forward
  klib-inverse
  klib-manipulator
  )

target_link_options(klib-demo PUBLIC
  --static
  )

