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
find_package(rostest REQUIRED)

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    add_rostest_gtest(tests_gripper test/gripper.test src/test/test_gripper.cpp)
#    target_link_libraries(tests_gripper ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#endif()

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
add_executable(simple_gripper src/Gripper.cpp src/gripper_action_server.cpp)
target_link_libraries(simple_gripper ${catkin_LIBRARIES})

#############
## Testing ##
#############

set(TEST_SRCS
        test/gripper_test.cpp
        test/main.cpp
)
#catkin_add_gtest()
    add_rostest_gtest(${PROJECT_NAME}-test ${TEST_SRCS})
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES})
endif()
