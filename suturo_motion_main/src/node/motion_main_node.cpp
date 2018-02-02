#include <ros/init.h>
#include "motion_main_node.h"

void start_node(int argc, char **argv) {
    ros::init(argc, argv, "suturo_motion_main");
    ros::NodeHandle nh;

}

MotionNode::MotionNode(const ros::NodeHandle &nh) :
node_handle(nh),
right_arm_group("right_arm"),
left_arm_group("left_arm"),
both_arms("arms"),
action_server(node_handle, "moving", boost::bind(&MotionNode::executeCommand, this, _1), false) {
    action_server.start();
}

void MotionNode::executeCommand(const motion_msgs::MovingCommandGoalConstPtr &goal) {
    moveit_msgs::MoveItErrorCodes error_code;
    geometry_msgs::PointStamped goal_point(goal->point_stamped);
    switch (goal->command) {
        case motion_msgs::MovingCommandGoal::MOVE_STANDARD_POSE :
            ROS_INFO("Starting to move to initial pose.");
            error_code = group_controller.moveArmsToInitial(both_arms);
            break;
        case motion_msgs::MovingCommandGoal::MOVE_RIGHT_ARM:
            ROS_INFO("Planning to move right arm to: ");
            error_code = group_controller.moveGroupToCoordinates(right_arm_group, goal_point);
            break;
        case motion_msgs::MovingCommandGoal::MOVE_LEFT_ARM:
            ROS_INFO("Planning to move left arm to: ");
            error_code = group_controller.moveGroupToCoordinates(left_arm_group, goal_point);
            break;
        default:
            ROS_ERROR("Got an unknown command constant. Can't do something. Make sure to call"
                              " the Service with the right constants from the msg file.");
            result.successful = false;
            action_server.setAborted(result, "UNKNOWN COMMAND - ABORTED");
            return;
    }
}
