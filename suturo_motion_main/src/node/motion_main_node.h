#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_msgs/MovingCommandAction.h>
#include "../movegroup/group_controller.h"

#ifndef SUTURO_MOTION_MAIN_MOTION_MAIN_NODE_H
#define SUTURO_MOTION_MAIN_MOTION_MAIN_NODE_H

class MotionNode {
private:
    ros::NodeHandle node_handle;
    actionlib::SimpleActionServer<motion_msgs::MovingCommandAction> action_server;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup::Plan execution_plan;
    moveit::planning_interface::MoveGroup right_arm_group;
    moveit::planning_interface::MoveGroup left_arm_group;
    moveit::planning_interface::MoveGroup both_arms;
    GroupController group_controller;
    motion_msgs::MovingCommandResult result;

public:
    MotionNode(const ros::NodeHandle &nh);

    void executeCommand(const motion_msgs::MovingCommandGoalConstPtr &goal);
};

void start_node(int argc, char **argv);

#endif //SUTURO_MOTION_MAIN_MOTION_MAIN_NODE_H