cmake_minimum_required(VERSION 3.26)
project(gravedadLuna)

set(CMAKE_CXX_STANDARD 17)

add_executable(gravedadLuna main.cpp)
find_package(box2d CONFIG REQUIRED)
target_link_libraries(gravedadLuna PRIVATE box2d::box2d)