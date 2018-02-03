#include "point_transformer.h"

geometry_msgs::PointStamped
PointTransformer::transformPointStamped(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& point) {
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

geometry_msgs::PoseStamped
PointTransformer::transformPoseStamped(const std::string& target_frame, const geometry_msgs::PoseStamped& pose) {
    geometry_msgs::PoseStamped pose_out;
    listener.transformPose(target_frame, pose, pose_out);
    return pose_out;
}