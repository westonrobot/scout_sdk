cmake_minimum_required(VERSION 3.0.0)
project(scout_base)

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Set compiler to use c++ 11 features
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Add libraries
set(SCOUT_BASE_SRC 
    src/scout_base.cpp
    # src/scout_serial.cpp
)
add_library(scoutbase STATIC ${SCOUT_BASE_SRC})
set_target_properties(scoutbase PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(scoutbase asyncio)
target_include_directories(scoutbase PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>	
    PRIVATE src)

 ## Add executables
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()