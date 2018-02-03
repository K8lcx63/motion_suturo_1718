#ifndef SUTURO_MOTION_MAIN_POINT_TRANSFORMER_H
#define SUTURO_MOTION_MAIN_POINT_TRANSFORMER_H


#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>

class PointTransformer {
private:
    tf::TransformListener listener;
public:
    /**
     * Transforms a given {@link geometry_msgs::PointStamped} to the matching PointStamped
     * in the planningframe of the given {@link moveit::planning_interface::MoveGroup}.
     *
     * @param group the group to whichs planning frame the point should be transformed.
     * @param point the point to transform.
     * @return the transformed {@link geometry_msgs::PointStamped}.
    */
    geometry_msgs::PointStamped
    transformPointStamped(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &point);

    /**
     * Transforms a {@link geometry_msgs::PoseStamped} into the given target_frame.
     *
     * @param target_frame The target frame to transform to.
     * @param pose The {@link geometry_msgs::PoseStamped} to transform.
     * @return the transformed pose stamped.
     */
    geometry_msgs::PoseStamped
    transformPoseStamped(const std::string &target_frame, const geometry_msgs::PoseStamped &pose);
};


#endif //SUTURO_MOTION_MAIN_POINT_TRANSFORMER_H
