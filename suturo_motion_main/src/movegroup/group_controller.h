#ifndef SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
#define SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H


#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>

class GroupController {
private:
    tf::TransformListener listener;
    moveit::planning_interface::MoveGroup::Plan execution_plan;

    geometry_msgs::PointStamped
    transformPointStamped(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &point);

public:
    /**
     * Moves the given {@link moveit::planning_interface::MoveGroup}
     * to the desired coordinates provided in {@link geometry_msgs::PointStamped}.
     *
     * @param group the group to move.
     * @param goal_point the coordinates to mvoe the group to.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement.
     */
    moveit_msgs::MoveItErrorCodes
    moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &goal_point);

    /**
     * Moves the given {@link moveit::planning_interface::MoveGroup} to the initial pose.
     *
     * @param group the group to move.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement.
     */
    moveit_msgs::MoveItErrorCodes moveArmsToInitial(moveit::planning_interface::MoveGroup &group);
};


#endif //SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
