#include "MoveGroupController.h"
#include "VisualizationMarkerPublisher.h"

tf::TransformListener transformListener;

struct MoveGroupController::Private {
    static geometry_msgs::PointStamped
    transform(const geometry_msgs::PointStamped &goal_point, const std::string &goal_frame) {
        geometry_msgs::PointStamped transformedPoint;
        ROS_INFO("Transforming Point from %s to %s", goal_point.header.frame_id.c_str(), goal_frame.c_str());
        transformListener.transformPoint(goal_frame, goal_point, transformedPoint);
        ROS_INFO("----Transformed point----");
        ROS_INFO("x %g", transformedPoint.point.x);
        ROS_INFO("y %g", transformedPoint.point.y);
        ROS_INFO("z %g", transformedPoint.point.z);
        return transformedPoint;
    }

    static geometry_msgs::PoseStamped createPose(const geometry_msgs::PointStamped &goal_point) {
        geometry_msgs::PoseStamped goalPose;
        goalPose.header.frame_id = goal_point.header.frame_id;
        goalPose.pose.position.x = goal_point.point.x;
        goalPose.pose.position.y = goal_point.point.y;
        goalPose.pose.position.z = goal_point.point.z;
        goalPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2,0,0);
        return goalPose;
    }
};

moveit_msgs::MoveItErrorCodes
MoveGroupController::moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, geometry_msgs::PointStamped &goal_point) {
    if (goal_point.header.frame_id != group.getPlanningFrame()) {
        goal_point = Private::transform(goal_point, group.getPlanningFrame());
    }
    geometry_msgs::PoseStamped goalPose = Private::createPose(goal_point);

    //publishVisualizationMarker(goal_point, COLOR_SCHEMA_KNOWLEDGE);

    group.setPoseTarget(goalPose);
    group.setGoalTolerance(0.05);

    //publishVisualizationMarker(point, COLOR_SCHEMA_MOTION);

    return group.move();
}
