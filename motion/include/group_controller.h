#ifndef SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
#define SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H

#include <geometry_msgs/PointStamped.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>
#include "point_transformer.h"
#include "visualization_marker.h"
#include <eigen_conversions/eigen_msg.h>
#include <motion_msgs/GripperAction.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include "planning_scene.h"

using namespace std;

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

    const string PATH_TO_GRIPPER_MESH = "package://knowledge_common/meshes/Gripper/Gripper.stl";

    ros::NodeHandle nodeHandle;
    ros::Publisher robotStatePublisher;
    ros::ServiceClient ikServiceClient;
    ros::Publisher beliefstatePublisherGrasp;
    ros::Publisher beliefstatePublisherDrop;

    moveit::planning_interface::MoveGroup::Plan execution_plan;
    PointTransformer point_transformer;
    VisualizationMarker visualizationMarker;
    actionlib::SimpleActionClient<motion_msgs::GripperAction> gripperclient;
    PlanningSceneController planning_scene_controller;

public:

    /**
     * Constructor.
     * @param nh Node Handle.
     */
    GroupController(const ros::NodeHandle &nh);

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
     * suitable for navigating the robot.
     *
     * @param group the group to move to the driving pose.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement.
     */
    moveit_msgs::MoveItErrorCodes moveArmsToDrivePose(moveit::planning_interface::MoveGroup &group);

    /**
     * Moves the given {@link moveit::planning_interface::MoveGroup} to a pose which is
     * suitable for navigating the robot while carrying an object.
     *
     * @param group the group to move to 'navigating-while-holding-object' pose.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the movement.
     */
    moveit_msgs::MoveItErrorCodes moveGroupToCarryingObjectPose(moveit::planning_interface::MoveGroup &group);

    /** Uses the given {@link moveit::planning_interface::MoveGroup} to poke the object, of which 
     * the center is given by {@link geometry_msgs::PointStamped}. 
     * @param group The group to take. 
     * @param object_middle The point at the center of the object to poke. 
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the poking action. 
     */ 
    moveit_msgs::MoveItErrorCodes pokeObject(moveit::planning_interface::MoveGroup& group,
                                             const geometry_msgs::PoseStamped& object_middle);
 
    /** 
     * Uses the given {@link moveit::planning_interface::MoveGroup} to grasp the object with the given grasp pose in
     * {@link geometry_msgs::PoseStamped}.
     * The {@link ros::Publisher} is for publishing a message to a topic after grasping
     * the object. This is needed for the beliefstate.
     * The string contains the label of the object to grasp.
     * @param group The group to take. 
     * @param objectGraspPoses The poses how the object may be grasped.
     * @param poseDescription describes the direction of the grasp pose for each pose given in object_grasp_poses
     * @param objectLabel The name of the object to grasp.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the grasping action. 
     */ 
    moveit_msgs::MoveItErrorCodes graspObject(moveit::planning_interface::MoveGroup& group,
                                              const geometry_msgs::PoseArray& objectGraspPoses, vector<string> poseDescription,
                                              double effort,
                                              std::string objectLabel);

    /**
     * Uses the given {@link moveit::planning_interface::MoveGroup} to drop the object with the given drop pose in
     * {@link geometry_msgs::PoseStamped}.
     * The {@link ros::Publisher} is for publishing a message to a topic after droping
     * the object. This is needed for the beliefstate.
     * The string contains the label of the object to drop.
     * @param group The group to take.
     * @param object_grasp_pose The pose how to drop the object.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the droping action.
     */
    moveit_msgs::MoveItErrorCodes dropObject(moveit::planning_interface::MoveGroup& group,
                                              const geometry_msgs::PoseStamped& object_drop_pose);

    /**
     * Checks if the object was successfully grasped.
     * @param gripperNum the number of the gripper to check.
     * @return true if the gripper has something in the gripper given by gripperNum.
     */
    bool checkIfObjectGraspedSuccessfully(int gripperNum);

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
