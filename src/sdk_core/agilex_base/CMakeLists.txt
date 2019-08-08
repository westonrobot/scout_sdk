## Add libraries
add_library(agilex_common INTERFACE)
# set_target_properties(agilex_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
# target_link_libraries(agilex_common asyncio)
target_include_directories(agilex_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>)

 ## Add executables
# if(BUILD_TESTS)
#     add_subdirectory(tests)
# endif()