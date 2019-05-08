# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(SOCKETCAN_LIB_SRC
	src/socketcan.c
)
add_library(socketcan STATIC ${SOCKETCAN_LIB_SRC})
# target_link_libraries(socketcan -lpthread)
target_include_directories(socketcan PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

