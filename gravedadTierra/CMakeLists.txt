cmake_minimum_required(VERSION 3.26)
project(gravedadTierra)

set(CMAKE_CXX_STANDARD 17)

add_executable(gravedadTierra main.cpp)
find_package(box2d CONFIG REQUIRED)
target_link_libraries(gravedadTierra PRIVATE box2d::box2d)