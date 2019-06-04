# Add executables
add_executable(test_asio_can test_asio_can.cpp)
target_link_libraries(test_asio_can socketcan)
