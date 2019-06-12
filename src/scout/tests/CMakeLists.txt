# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## Unit tests with GoogleTest
set(GTEST_SRC
    gtests/scout_can_protocol_test.cpp
)
add_executable(scout_sdk_test ${GTEST_SRC})
target_link_libraries(scout_sdk_test gtest gtest_main scout_sdk)
target_include_directories(scout_sdk_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GTEST_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>)

# tests
# add_executable(test_scout_cpp test_scout_cpp.cpp)
# target_link_libraries(test_scout_cpp scout stopwatch)

add_executable(test_scout_base test_scout_base.cpp)
target_link_libraries(test_scout_base scout_sdk)

add_executable(test_can_msg test_can_msg.cpp)
target_link_libraries(test_can_msg scout_sdk)
