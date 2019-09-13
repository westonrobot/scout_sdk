# Add source directories
add_subdirectory(common)
add_subdirectory(scout_base)
# add_subdirectory(hunter_base)

if(BUILD_TESTS)
    add_subdirectory(unit_tests)
endif()