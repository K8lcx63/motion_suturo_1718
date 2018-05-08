#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <planning_scene.h>

TEST(PlanningSceneTest, TestRemoveObjectEnvFalse) {
    PlanningSceneController planningSceneController;

    bool result = planningSceneController.removeObjectFromEnvironment("");

    ASSERT_FALSE(result);
}