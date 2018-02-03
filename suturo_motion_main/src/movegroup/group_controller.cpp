#include "group_controller.h"

/**
 * Transforms a given {@link geometry_msgs::PointStamped} to the matching PointStamped
 * in the planningframe of the given {@link moveit::planning_interface::MoveGroup}.
 *
 * @param group the group to whichs planning frame the point should be transformed.
 * @param point the point to transform.
 * @return the transformed {@link geometry_msgs::PointStamped}.
 */
geometry_msgs::PointStamped
GroupController::transformPointStamped(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& point) {
    if (group.getPlanningFrame() == point.header.frame_id) {
        ROS_INFO("TRANSFORMING POINT");
        ROS_INFO("from: %s to: %s", point.header.frame_id.c_str(), group.getPlanningFrame().c_str());
        geometry_msgs::PointStamped return_point;
        listener.transformPoint(group.getPlanningFrame(), point, return_point);
        return return_point;
    }
    ROS_INFO("POINT ALREADY IN RIGHT FRAME");
    return point;
}

moveit_msgs::MoveItErrorCodes
GroupController::moveGroupToCoordinates(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& goal_point) {
    geometry_msgs::PointStamped point = transformPointStamped(group, goal_point);

    //publishVisualizationMarker(goal_point, COLOR_SCHEMA_KNOWLEDGE);

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