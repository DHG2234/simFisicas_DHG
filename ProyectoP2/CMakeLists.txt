cmake_minimum_required(VERSION 3.26)
project(ProyectoP2)

set(CMAKE_CXX_STANDARD 17)

add_executable(ProyectoP2 main.cpp)

find_package(box2d CONFIG REQUIRED)
target_link_libraries(ProyectoP2 PRIVATE box2d::box2d)

find_package(SFML COMPONENTS system window graphics CONFIG REQUIRED)

target_link_libraries(ProyectoP2 PRIVATE sfml-system sfml-network sfml-graphics sfml-window)

# If you want SFML to provide an implementation of main():
target_link_libraries(ProyectoP2 PRIVATE sfml-main)