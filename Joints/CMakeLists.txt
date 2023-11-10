cmake_minimum_required(VERSION 3.26)
project(Joints)

set(CMAKE_CXX_STANDARD 17)

# Agrega el ejecutable
add_executable(Joints main.cpp)

find_package(box2d CONFIG REQUIRED)
find_package(SFML COMPONENTS system window graphics CONFIG REQUIRED)

target_link_libraries(Joints PRIVATE sfml-system sfml-network sfml-graphics sfml-window)

# If you want SFML to provide an implementation of main():
target_link_libraries(Joints PRIVATE sfml-main)
target_link_libraries(Joints PRIVATE box2d::box2d)