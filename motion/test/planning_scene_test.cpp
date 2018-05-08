#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <planning_scene.h>


TEST(PlanningSceneTest, TestRemoveObjectEnvFalse) {
    ros::NodeHandle nh;
    PlanningSceneController planningSceneController(nh);

    bool result = planningSceneController.removeObjectFromEnvironment("");

    ASSERT_FALSE(result);
}

TEST(PlanningSceneTest, TestDetachObjectNoName) {
    ros::NodeHandle nh;
    PlanningSceneController planningSceneController(nh);

    bool result = planningSceneController.detachObject("", "");

    ASSERT_FALSE(result);
}

