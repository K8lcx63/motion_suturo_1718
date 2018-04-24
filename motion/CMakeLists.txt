cmake_minimum_required(VERSION 2.8.3)
project(motion)

add_definitions(-std=c++11)

# temprorary ignored warnings
SET(CMAKE_CXX_FLAGS "-w")


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  gmapping
  iai_pr2
  knowledge_msgs
  message_generation
  motion_msgs
  moveit_ros_planning_interface
  roscpp
  std_msgs
  geometric_shapes
  tf
  visualization_msgs
  marker_publisher
  pr2_controllers_msgs
  motion_gripper
  sensor_msgs
  tf_conversions
)

find_package(Boost REQUIRED COMPONENTS
    system
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    tf
    motion_gripper
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

#create executables
add_executable(
    motion
    src/main.cpp
    src/visualization/visualization_marker.cpp
    src/include/visualization/visualization_marker.h
    src/transformer/point_transformer.cpp
    src/include/transform/point_transformer.h
    src/planningscene/planning_scene.cpp
    src/include/planningscene/planning_scene.h
    src/node/motion_main_node.cpp
    src/include/node/motion_main_node.h
    src/movegroup/group_controller.cpp
    src/include/movegroup/group_controller.h
)

add_executable(
	move_base 
	src/movebase/move_base.cpp
)

add_executable(
  test_client
  src/test_client.cpp
)

target_link_libraries(
    motion
    ${catkin_LIBRARIES}
)

target_link_libraries(
    move_base
    ${catkin_LIBRARIES}
)

target_link_libraries(
    test_client
    ${catkin_LIBRARIES}
)