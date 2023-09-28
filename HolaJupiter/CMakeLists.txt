cmake_minimum_required(VERSION 3.26)
project(HolaJupiter)

set(CMAKE_CXX_STANDARD 17)

add_executable(HolaJupiter main.cpp)
find_package(box2d CONFIG REQUIRED)
target_link_libraries(HolaJupiter PRIVATE box2d::box2d)