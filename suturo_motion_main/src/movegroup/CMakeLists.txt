cmake_minimum_required(VERSION 3.8)
project(movegroup)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        group_controller.cpp)

add_executable(movegroup ${SOURCE_FILES})