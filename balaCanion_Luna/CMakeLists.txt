cmake_minimum_required(VERSION 3.26)
project(balaCanion_Luna)

set(CMAKE_CXX_STANDARD 17)

add_executable(balaCanion_Luna main.cpp)
find_package(box2d CONFIG REQUIRED)
target_link_libraries(balaCanion_Luna PRIVATE box2d::box2d)