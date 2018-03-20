#ifndef SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
#define SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H

#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>
#include "../transform/point_transformer.h"
#include "../visualization/visualization_marker.h"
#include <eigen_conversions/eigen_msg.h>
#include <motion_msgs/GripperAction.h>
#include <actionlib/client/simple_action_client.h>

/**
 * Class to control movement of moveit MoveGroups.
 */
class GroupController {
private:
    const float GRIPPER_LENGTH_RIGHT = 0.21f;
    const float GRIPPER_LENGTH_LEFT = 0.24f;
    const float DISTANCE_BEFORE_POKING = 0.03f; 
    const float TABLE_HEIGHT = 0.84f;
    const float MAXIMUM_OBJECT_HEIGHT = 0.30f;
    const float DISTANCE_BEFORE_GRASPING = 0.03f; 

    moveit::planning_interface::MoveGroup::Plan execution_plan;
    PointTransformer point_transformer;
    VisualizationMarker visualizationMarker;
    actionlib::SimpleActionClient<motion_msgs::GripperAction> gripperclient;

public:
    /**
     * Constructor.
     */
    GroupController();

    /** 
     * Moves the given {@link moveit::planning_interface::MoveGroup} 
     * to the desired pose provided in {@link geometry_msgs::PoseStamped}. 
     * 
     * @param group the group to move. 
     * @param goal_pose the pose to move the group to. 
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement. 
     */ 
    moveit_msgs::MoveItErrorCodes 
    moveGroupToPose(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PoseStamped &goal_pose);

    /**
     * Moves the given {@link moveit::planning_interface::MoveGroup} to a pose which is
     * suitable for driving the robot.
     *
     * @param group the group to move to pose.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement.
     */
    moveit_msgs::MoveItErrorCodes moveArmsToDrivePose(moveit::planning_interface::MoveGroup &group);

    /**
     * Moves the given {@link moveit::planning_interface::MoveGroup} to a pose which is
     * suitable for driving the robot while carrying an object.
     *
     * @param group the group to move to pose.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement.
     */
    moveit_msgs::MoveItErrorCodes moveArmsToCarryingObjectPose(moveit::planning_interface::MoveGroup &group);

    /** Uses the given {@link moveit::planning_interface::MoveGroup} to poke the object, of which 
     * the center is given by {@link geometry_msgs::PointStamped}. 
     * @param group The group to take. 
     * @param object_middle The point at the center of the object to poke. 
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the poking action. 
     */ 
    moveit_msgs::MoveItErrorCodes pokeObject(moveit::planning_interface::MoveGroup& group,
                                             const geometry_msgs::PoseStamped& object_middle);
 
    /** 
     * Uses the given {@link moveit::planning_interface::MoveGroup} to grasp/drop the object, of which
     * the pose to grasp/drop is given by {@link geometry_msgs::PoseStamped}.
     * The bool states out, whether to releaseObject. If false, object get's grasped.
     * The {@link ros::Publisher} is for publishing a message to a topic after grasping/
     * releasing an object. This is needed for the beliefstate.
     * The string contains the label of the object to grasp/release.
     * @param group The group to take. 
     * @param object_grasp_pose The pose how to grasp the object.
     * @param releaseObject True, if object shell be released. False if it shell be grasped.
     * @param beliefstatePublisher Publisher to publish message for beliefstatetopic.
     * @param objectLabel The name of the object to grasp/release.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the grasping action. 
     */ 
    moveit_msgs::MoveItErrorCodes graspObject(moveit::planning_interface::MoveGroup& group,
                                              const geometry_msgs::PoseStamped& object_grasp_pose, float effort, bool releaseObject,
                                                ros::Publisher beliefstatePublisher, std::string objectLabel);
 
    /** 
     * Opens the gripper given by gripperName. 
     * @param gripperNum the number of the gripper to close.
     */
    void openGripper(int gripperNum);

    /**
     * Closes the gripper given by gripperName.
     * @param gripperNum the number of the gripper to close.
     */
    void closeGripper(int gripperNum, float& effort);
};


#endif //SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
