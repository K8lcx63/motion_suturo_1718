cmake_minimum_required(VERSION 2.8.3)
project(motion_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
)

add_action_files(
   DIRECTORY action   
   FILES
   MovingCommand.action
 )

generate_messages(
   DEPENDENCIES
   geometry_msgs
   actionlib_msgs
)


catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

