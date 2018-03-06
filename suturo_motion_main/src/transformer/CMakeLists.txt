cmake_minimum_required(VERSION 3.8)
project(transformer)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        point_transformer.cpp)

add_executable(transformer ${SOURCE_FILES})