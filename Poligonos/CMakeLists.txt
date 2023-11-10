cmake_minimum_required(VERSION 3.26)
project(Poligonos)

set(CMAKE_CXX_STANDARD 17)

add_executable(Poligonos main.cpp)
find_package(box2d CONFIG REQUIRED)
target_link_libraries(Poligonos PRIVATE box2d::box2d)