cmake_minimum_required(VERSION 3.8)
project(node)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        motion_main_node.cpp)

add_executable(node ${SOURCE_FILES})