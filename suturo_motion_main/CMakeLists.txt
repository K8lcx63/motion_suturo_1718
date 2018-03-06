cmake_minimum_required(VERSION 2.8.3)
project(suturo_motion_main)

add_definitions(-std=c++11)

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
  tf
  visualization_msgs
  marker_publisher
  pr2_controllers_msgs
  motion_gripper
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
        suturo_motion_main
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

target_link_libraries(
        suturo_motion_main
        ${catkin_LIBRARIES}
)