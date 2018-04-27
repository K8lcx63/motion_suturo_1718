#include <point_transformer.h>

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

geometry_msgs::Pose
PointTransformer::lookupTransformPose(const std::string& target_frame, const std::string& source_frame, const ros::Time& time){
    tf::StampedTransform result;
    geometry_msgs::Pose toReturn;

    listener.lookupTransform (target_frame, source_frame, time, result);

    // create geometry_msgs::Pose from tf::StampedTransform
    toReturn.position.x = result.getOrigin().x();
    toReturn.position.y = result.getOrigin().y();
    toReturn.position.z = result.getOrigin().z();
    toReturn.orientation.x = result.getRotation().x();
    toReturn.orientation.y = result.getRotation().y();
    toReturn.orientation.z = result.getRotation().z();
    toReturn.orientation.w = result.getRotation().w();

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

    // reset stamp in header to take latest data available
    geometry_msgs::PoseStamped poseStampedLatest = pose;
    poseStampedLatest.header.stamp = ros::Time(0);

    listener.transformPose(target_frame, poseStampedLatest, pose_out);

    return pose_out;
}