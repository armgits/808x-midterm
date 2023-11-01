# Create a library called "klib-datatypes" (in Linux, this library is created
# with the name of either libklib-datatypes.a or klib-datatypes.so).
add_library (klib-datatypes INTERFACE)

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(klib-datatypes INTERFACE .)
