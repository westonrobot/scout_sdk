## Add libraries
add_library(agilex_protocol INTERFACE)
target_include_directories(agilex_protocol INTERFACE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
$<INSTALL_INTERFACE:include>)
