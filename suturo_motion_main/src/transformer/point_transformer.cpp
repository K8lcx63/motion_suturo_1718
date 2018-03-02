#include "../include/transform/point_transformer.h"

geometry_msgs::PointStamped
PointTransformer::lookupTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time){
    tf::StampedTransform result;
    geometry_msgs::PointStamped toReturn;

    listener.lookupTransform (target_frame, source_frame, time, result);

    // create geometry_msgs::PointStamped from tf::StampedTransform
    tf::Vector3 positionInTargetFrame (0,0,0);
    tf::Vector3 targetFramePosition = result * positionInTargetFrame;

    toReturn.header.seq++;
    toReturn.header.stamp = ros::Time::now();
    toReturn.header.frame_id = source_frame;
    toReturn.point.x = targetFramePosition.getX();
    toReturn.point.y = targetFramePosition.getY();
    toReturn.point.z = targetFramePosition.getZ();

    return toReturn;
}

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