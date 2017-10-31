cmake_minimum_required(VERSION 3.8)
project(catkin_ws)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/main.cpp)

add_executable(catkin_ws ${SOURCE_FILES})