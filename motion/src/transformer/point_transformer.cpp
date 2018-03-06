#include "../include/transform/point_transformer.h"

geometry_msgs::PointStamped 
PointTransformer::lookupTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time){ 
    tf::StampedTransform result; 
    geometry_msgs::PointStamped toReturn; 
 
    listener.lookupTransform (target_frame, source_frame, time, result); 
 
    // create geometry_msgs::PointStamped from tf::StampedTransform
    toReturn.header.seq++; 
    toReturn.header.stamp = ros::Time::now(); 
    toReturn.header.frame_id = target_frame;
    toReturn.point.x = result.getOrigin().x();
    toReturn.point.y = result.getOrigin().y();
    toReturn.point.z = result.getOrigin().z();

    return toReturn; 
}

geometry_msgs::PointStamped
PointTransformer::transformPointStamped(moveit::planning_interface::MoveGroup &group,
                                        const geometry_msgs::PointStamped &point) {
    return transformPointStamped(group.getPlanningFrame(), point);
}

geometry_msgs::PointStamped PointTransformer::transformPointStamped(const std::string& target_frame, const geometry_msgs::PointStamped& point) {
    if (target_frame == point.header.frame_id) {
        ROS_INFO("TRANSFORMING POINT");
        ROS_INFO("from: %s to: %s", point.header.frame_id.c_str(), target_frame.c_str());
        geometry_msgs::PointStamped return_point;
        listener.transformPoint(target_frame, point, return_point);
        return return_point;
    }
    ROS_INFO("POINT ALREADY IN RIGHT FRAME");
    return point;
}


geometry_msgs::PoseStamped
PointTransformer::transformPoseStamped(const std::string &target_frame, const geometry_msgs::PoseStamped &pose) {
    geometry_msgs::PoseStamped pose_out;
    listener.waitForTransform(target_frame, pose.header.frame_id, ros::Time(0), ros::Duration(5));
    listener.transformPose(target_frame, pose, pose_out);

    geometry_msgs::PointStamped toVisualize;
    toVisualize.header.frame_id = target_frame;
    toVisualize.point = pose_out.pose.position;

    return pose_out;
}