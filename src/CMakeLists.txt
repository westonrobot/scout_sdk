# Add source directories
add_subdirectory(demo)
add_subdirectory(scout)
add_subdirectory(driver)
add_subdirectory(third_party)

add_subdirectory(monitor)

if(BUILD_TESTS)
    add_subdirectory(unit_tests)
endif()