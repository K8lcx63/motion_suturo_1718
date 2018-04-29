#ifndef SUTURO_MOTION_MAIN_POINT_TRANSFORMER_H
#define SUTURO_MOTION_MAIN_POINT_TRANSFORMER_H


#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>
#include <visualization_marker.h>

/**
 * Class for transforming PointStampeds and PoseStampeds.
 */
class PointTransformer {
private:
    tf::TransformListener listener;
public:

    /** Looks up a given target frame in a given source frame at a given {@link ros::Time}. 
     * 
     * @param target_frame the frame to be looked up. 
     * @param source_frame the frame in which the looked up frame should be given back. 
     * @param time the time at which the frame should be looked up. 
     * @return the target frame in the source frame at time in form of a {@link geometry_msgs::PointStamped}. 
    */ 
    geometry_msgs::PointStamped 
    lookupTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time);

    /** Looks up a given target frame in a given source frame at a given {@link ros::Time}.
     *
     * @param target_frame the frame to be looked up.
     * @param source_frame the frame in which the looked up frame should be given back.
     * @param time the time at which the frame should be looked up.
     * @return the target frame in the source frame at time in form of a {@link geometry_msgs::Pose}.
    */
    geometry_msgs::Pose
    lookupTransformPose(const std::string& target_frame, const std::string& source_frame, const ros::Time& time);

    /**
     * Transforms a given {@link geometry_msgs::PointStamped} to the matching PointStamped
     * in the planningframe of the given {@link moveit::planning_interface::MoveGroup}.
     *
     * @param group the group to whichs planning frame the point should be transformed.
     * @param point the point to transform.
     * @return the transformed {@link geometry_msgs::PointStamped}.
    */
    geometry_msgs::PointStamped
    transformPointStamped(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& point);

    /**
     * Transforms a given {@link geometry_msgs::PointStamped} to the matching PointStamped
     * in the given target_frame.
     *
     * @param target_frame the frame to transform the point to.
     * @param point the point to transform.
     * @return the transformed {@link geometry_msgs::PointStamped}.
    */
    geometry_msgs::PointStamped transformPointStamped(const std::string& target_frame, const geometry_msgs::PointStamped& point);

    /**
     * Transforms a {@link geometry_msgs::PoseStamped} into the given target_frame.
     *
     * @param target_frame The target frame to transform to.
     * @param pose The {@link geometry_msgs::PoseStamped} to transform.
     * @return the transformed pose stamped.
     */
    geometry_msgs::PoseStamped
    transformPoseStamped(const std::string &target_frame, const geometry_msgs::PoseStamped &pose);

    /**
     * Transforms a {@link geometry_msgs::Pose} goal, given for the end effector frame ('r_gripper_tool_frame' or
     * 'l_gripper_tool_frame') to a goal pose for the wrist frame ('r_wrist_roll_link' or 'l_wrist_roll_link'), depending on the
     * {@link moveit::planning_interface::MoveGroup}.
     * This function is required, because the {@link moveit::planning_interface::MoveGroup} we use plans the motion
     * with the wrist link, but the grasp poses are given for the end effector link.
     *
     * @param poseForEndEffector the pose the end effector shall be moved to.
     * @param group the group of which the end effector shall be moved to poseForEndEffector.
     * @return the pose the wrist frame of the group has to be moved to, to reach the pose poseForEndEffector for the
     *         end effector frame.
     */
    geometry_msgs::PoseStamped transformPoseFromEndEffectorToWristFrame (const geometry_msgs::PoseStamped& poseForEndEffector,
                                                                         moveit::planning_interface::MoveGroup& group);

    static void
    copyStampedTransformToPose(tf::StampedTransform &toTransform, geometry_msgs::Pose &result) {
        result.position.x = toTransform.getOrigin().x();
        result.position.y = toTransform.getOrigin().y();
        result.position.z = toTransform.getOrigin().z();
        result.orientation.x = toTransform.getRotation().x();
        result.orientation.y = toTransform.getRotation().y();
        result.orientation.z = toTransform.getRotation().z();
        result.orientation.w = toTransform.getRotation().w();
    }
};


#endif //SUTURO_MOTION_MAIN_POINT_TRANSFORMER_H
