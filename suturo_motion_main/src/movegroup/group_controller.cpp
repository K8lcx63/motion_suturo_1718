#include "group_controller.h"

moveit_msgs::MoveItErrorCodes
GroupController::moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &goal_point) {
    geometry_msgs::PointStamped point;

    //publishVisualizationMarker(goal_point, COLOR_SCHEMA_KNOWLEDGE);
    geometry_msgs::PointStamped tempPoint;
    tempPoint.header = goal_point.header;
    tempPoint.point.x = goal_point.point.x;
    tempPoint.point.y = goal_point.point.y;
    tempPoint.point.z = goal_point.point.z;
    ROS_INFO("Transforming Point from %s to %s", goal_point.header.frame_id.c_str(), group.getPlanningFrame().c_str());
    listener.transformPoint(group.getPlanningFrame(), tempPoint, point);
    ROS_INFO("----Transformed point----");
    ROS_INFO("x %g", point.point.x);
    ROS_INFO("y %g", point.point.y);
    ROS_INFO("z %g", point.point.z);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = point.header.frame_id;
    poseStamped.pose.position.x = point.point.x;
    poseStamped.pose.position.y = point.point.y;
    poseStamped.pose.position.z = point.point.z;
    poseStamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2,0,0);
    group.setPoseTarget(poseStamped);
    group.setGoalTolerance(0.05);

    //group.setPositionTarget(point.point.x, point.point.y, point.point.z);
    //publishVisualizationMarker(point, COLOR_SCHEMA_MOTION);


    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);


    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        error_code = group.move();
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