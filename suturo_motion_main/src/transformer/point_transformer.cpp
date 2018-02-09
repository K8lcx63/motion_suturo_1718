#include "../include/transform/point_transformer.h"

geometry_msgs::PointStamped
PointTransformer::transformPointStamped(moveit::planning_interface::MoveGroup &group,
                                        const geometry_msgs::PointStamped &point) {
    return transformPointStamped(group.getPlanningFrame(), point);
}

geometry_msgs::PointStamped PointTransformer::transformPointStamped(const std::string& target_frame, const geometry_msgs::PointStamped& point) {
    if (target_frame == point.header.frame_id) {
        visualizationMarker.publishVisualizationMarkerWithColor(point, "beforeTransform", Color::GREEN);
        ROS_INFO("TRANSFORMING POINT");
        ROS_INFO("from: %s to: %s", point.header.frame_id.c_str(), target_frame.c_str());
        geometry_msgs::PointStamped return_point;
        listener.transformPoint(target_frame, point, return_point);
        visualizationMarker.publishVisualizationMarkerWithColor(return_point, "motion", Color::RED);
        return return_point;
    }
    ROS_INFO("POINT ALREADY IN RIGHT FRAME");
    visualizationMarker.publishVisualizationMarkerWithColor(point, "motion", Color::RED);
    return point;
}


geometry_msgs::PoseStamped
PointTransformer::transformPoseStamped(const std::string &target_frame, const geometry_msgs::PoseStamped &pose) {
    geometry_msgs::PoseStamped pose_out;
    listener.transformPose(target_frame, pose, pose_out);
    return pose_out;
}