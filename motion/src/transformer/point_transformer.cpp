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
    listener.lookupTransform(target_frame, source_frame, time, result);

    // create geometry_msgs::Pose from tf::StampedTransform
    geometry_msgs::Pose toReturn;
    copyStampedTransformToPose(result, toReturn);

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

geometry_msgs::PoseStamped
PointTransformer::transformPoseFromEndEffectorToWristFrame (const geometry_msgs::PoseStamped& poseForEndEffector,
                                                            moveit::planning_interface::MoveGroup& group)
{

    // create pose from 'poseForEndEffector' by transform it into map space
    geometry_msgs::PoseStamped poseStampedForEndEffectorInWorld;
    listener.transformPose("map", poseForEndEffector, poseStampedForEndEffectorInWorld);

    geometry_msgs::Pose poseForEndEffectorInWorld = poseStampedForEndEffectorInWorld.pose;

    
    //get the tool and wrist frame depending on used group
    std::string toolFrame = (group.getName() == "right_arm") ? "r_gripper_tool_frame" : "l_gripper_tool_frame";
    std::string wristFrame = group.getEndEffectorLink();

    tf::Transform worldToWristTF, worldToToolTF;
    tf::StampedTransform toolToWristTFStamped;
    geometry_msgs::Pose worldToWrist;

    // lookup transform from tool frame to wrist frame
    listener.lookupTransform(toolFrame, wristFrame, ros::Time(0), toolToWristTFStamped);
    tf::Transform toolToWristTF (toolToWristTFStamped.getRotation(), toolToWristTFStamped.getOrigin());

    // convert to TF data type, so math operations can be applied
    tf::poseMsgToTF(poseForEndEffectorInWorld, worldToToolTF);

    // transform from tool to wrist and save result in 'worldToWrist'
    worldToWristTF = worldToToolTF * toolToWristTF;

    // convert TF data type back to geometry_msgs type
    tf::poseTFToMsg(worldToWristTF, worldToWrist);

    // create PoseStamped object to return
    geometry_msgs::PoseStamped toReturn;
    toReturn.header.frame_id = "map";
    toReturn.header.stamp = ros::Time(0);
    toReturn.pose = worldToWrist;

    return toReturn;
}

void PointTransformer::copyStampedTransformToPose(tf::StampedTransform &toTransform, geometry_msgs::Pose &result) {
    result.position.x = toTransform.getOrigin().x();
    result.position.y = toTransform.getOrigin().y();
    result.position.z = toTransform.getOrigin().z();
    result.orientation.x = toTransform.getRotation().x();
    result.orientation.y = toTransform.getRotation().y();
    result.orientation.z = toTransform.getRotation().z();
    result.orientation.w = toTransform.getRotation().w();
}