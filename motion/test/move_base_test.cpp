#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include "../src/movebase/move_base.cpp"

TEST(MoveBaseTest, TestGenerateMessage) {
    Torso torso;
    pr2_controllers_msgs::SingleJointPositionGoal result = torso.generateMsg(0.2, ros::Duration(1.0), 0.5);

    ASSERT_EQ(result.position, 0.2);
    ASSERT_EQ(result.max_velocity, 0.5);
    ASSERT_EQ(result.min_duration, ros::Duration(1.0));
}