cmake_minimum_required(VERSION 3.0.0)
project(scout)

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Add source directories
add_subdirectory(scout_cpp)
add_subdirectory(scout_io)
add_subdirectory(scout_io_v2)

## Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()