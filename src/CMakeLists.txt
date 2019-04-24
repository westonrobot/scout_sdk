# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/src/cmake/modules")

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# Add source directories
add_subdirectory(demo)
add_subdirectory(scout_cpp)
add_subdirectory(scout_io)
add_subdirectory(third_party)
