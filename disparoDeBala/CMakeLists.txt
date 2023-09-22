cmake_minimum_required(VERSION 3.26)
project(disparoDeBala)

set(CMAKE_CXX_STANDARD 17)

add_executable(disparoDeBala main.cpp)
find_package(box2d CONFIG REQUIRED)
target_link_libraries(disparoDeBala PRIVATE box2d::box2d)