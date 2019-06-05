# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
# add_executable(test_scout_cpp test_scout_cpp.cpp)
# target_link_libraries(test_scout_cpp scout stopwatch)

add_executable(test_scout_base test_scout_base.cpp)
target_link_libraries(test_scout_base scoutbase)

add_executable(test_can_protocol test_can_protocol.cpp)
target_link_libraries(test_can_protocol scoutbase)