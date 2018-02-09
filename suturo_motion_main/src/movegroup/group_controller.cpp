#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "group_controller.h"
#include "../visualization/visualization_marker.h"

moveit_msgs::MoveItErrorCodes
GroupController::moveGroupToCoordinates(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& goal_point) {
    geometry_msgs::PointStamped point = point_transformer.transformPointStamped(group, goal_point);


    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    //publishVisualizationMarker(goal_point, COLOR_SCHEMA_KNOWLEDGE);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = point.header.frame_id;
    poseStamped.pose.position.x = point.point.x;
    poseStamped.pose.position.y = point.point.y;
    poseStamped.pose.position.z = point.point.z;
    poseStamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2,0,0);
    group.setPoseTarget(poseStamped);
    group.setGoalTolerance(0.02);

    //group.setPositionTarget(point.point.x, point.point.y, point.point.z);
    visualizationMarker.publishVisualizationMarker(point, "motion");
    //publishVisualizationMarker(point, COLOR_SCHEMA_MOTION);


    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);


    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

        robot_trajectory::RobotTrajectory trajectory (kinematic_model, group.getName());
        trajectory.setRobotTrajectoryMsg(*(group.getCurrentState()), execution_plan.trajectory_);

        moveit::core::RobotState robotState = trajectory.getLastWayPoint();
        Eigen::Affine3d eef_transform = robotState.getGlobalLinkTransform(group.getEndEffectorLink());

        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(eef_transform, pose);

        pose.position.x -= 0.18f;
        group.setPoseTarget(pose);

        error_code = group.plan(execution_plan);

        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            group.move();
        }
    }
    return error_code;
}

moveit_msgs::MoveItErrorCodes GroupController::moveArmsToInitial(moveit::planning_interface::MoveGroup &group) {
    group.setNamedTarget("arms_initial");

    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);

    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        error_code = group.move();
    }
    return error_code;
}
