## Add libraries
# set(AGILEX_PROTO_SRC 
#     src/scout_can_parser.cpp
#     src/scout_serial_parser.cpp
#     src/hunter_can_parser.cpp
#     src/hunter_serial_parser.cpp
# )
add_library(agilex_protocol INTERFACE)
# set_target_properties(agilex_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
# target_link_libraries(agilex_protocol asyncio)
target_include_directories(agilex_protocol INTERFACE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
$<INSTALL_INTERFACE:include>)

## Add executables
# if(BUILD_TESTS)
#     add_subdirectory(tests)
# endif()