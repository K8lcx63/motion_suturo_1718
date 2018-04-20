#include <gtest/gtest.h>
#include <Gripper.h>

TEST(MaxTest, gripper_test) {
    ASSERT_TRUE(true);
}

TEST(GripperTest, test_message_generation) {
    pr2_controllers_msgs::Pr2GripperCommandGoal result = Gripper::generateGripperGoal(1.1, 5.5);
    ASSERT_EQ(result.command.position, 1.1f);
    ASSERT_EQ(result.command.max_effort, 5.5f);
}