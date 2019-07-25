# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(demo_scout_can demo_scout_can.cpp)
target_link_libraries(demo_scout_can scoutbase)

add_executable(demo_scout_serial demo_scout_serial.cpp)
target_link_libraries(demo_scout_serial scoutbase)