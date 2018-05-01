#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "motiontest");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}