# Dependency libraries
# find_package(LIB_NAME REQUIRED)

# Add libraries
set(ASYNC_SERIAL_LIB_SRC
	src/async_serial.cpp
)
add_library(asyncserial STATIC ${ASYNC_SERIAL_LIB_SRC})
target_link_libraries(asyncserial asio pthread)
target_include_directories(asyncserial PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
# if(BUILD_TESTS)
	add_subdirectory(tests)
# endif()