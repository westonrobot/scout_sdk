# Dependency libraries
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

# Add libraries
set(SCOUT_MONITOR_SRC
    src/scout_monitor.cpp
)
add_library(monitor STATIC ${SCOUT_MONITOR_SRC})
target_link_libraries(monitor scout_sdk ${CURSES_LIBRARIES})
# target_compile_definitions(monitor PUBLIC "-DASIO_ENABLE_OLD_SERVICES -DASIO_HAS_POSIX_STREAM_DESCRIPTOR")
target_include_directories(monitor PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CURSES_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()