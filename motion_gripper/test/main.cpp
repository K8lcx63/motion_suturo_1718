#include <gtest/gtest.h>
#include <ros/node_handle.h>

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    //ros::init(argc, argv, "gripper_test_node");
    //ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}