# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(demo_scout_cpp demo_scout_cpp.cpp)
target_link_libraries(demo_scout_cpp scoutcpp stopwatch)

## Find LCM
find_package(LCM REQUIRED)
if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

add_executable(demo_scout_lcm demo_scout_lcm.cpp)
target_link_libraries(demo_scout_lcm scoutcpp stopwatch ${LCM_LIBRARIES})
target_include_directories(demo_scout_lcm INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:include>)