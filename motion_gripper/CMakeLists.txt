cmake_minimum_required(VERSION 2.8.3)
project(motion_gripper)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        actionlib
        pr2_controllers_msgs
)
find_package(Boost REQUIRED COMPONENTS
       system
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_gripper
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#create executables
add_executable(simple_gripper src/simple_gripper.cpp)
target_link_libraries(simple_gripper ${catkin_LIBRARIES})