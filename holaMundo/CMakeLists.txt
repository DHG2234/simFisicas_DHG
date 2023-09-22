cmake_minimum_required(VERSION 3.26)
project(holaMundo)

set(CMAKE_CXX_STANDARD 17)

add_executable(holaMundo main.cpp)
find_package(box2d CONFIG REQUIRED)
target_link_libraries(holaMundo PRIVATE box2d::box2d)