cmake_minimum_required(VERSION 3.10)
project(src)
set(CMAKE_CXX_STANDARD 14)
add_executable(src
        ultra_low_latency.cpp)
target_link_libraries(src PRIVATE ws2_32)