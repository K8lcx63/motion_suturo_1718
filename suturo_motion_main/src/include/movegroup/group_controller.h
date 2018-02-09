#ifndef SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
#define SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H


#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>
#include "../transform/point_transformer.h"
#include "../visualization/visualization_marker.h"
#include <eigen_conversions/eigen_msg.h>

/**
 * Class to controlle movement of moveit MoveGroups.
 */
class GroupController {
private:
    const float GRIPPER_LENGTH_RIGHT = 0.15f;
    const float GRIPPER_LENGTH_LEFT = 0.18f;

    moveit::planning_interface::MoveGroup::Plan execution_plan;
    PointTransformer point_transformer;
    VisualizationMarker visualizationMarker;

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

    /**
     * Moves the given {@link moveit::planning_interface::MoveGroup} with the gripper as end effector.
     * @param group The group to move.
     * @param goal_point The point to move the group to.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement.
     */
    moveit_msgs::MoveItErrorCodes moveEndEffectorToGoal(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& goal_point);
};


#endif //SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
