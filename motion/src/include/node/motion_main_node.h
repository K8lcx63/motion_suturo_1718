#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_msgs/MovingCommandAction.h>
#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <knowledge_msgs/GraspObject.h>
#include <knowledge_msgs/DropObject.h>
#include <knowledge_msgs/Gripper.h>
#include <knowledge_msgs/PerceivedObjectBoundingBox.h>
#include "../movegroup/group_controller.h"
#include "../planningscene/planning_scene.h"

#ifndef SUTURO_MOTION_MAIN_MOTION_MAIN_NODE_H
#define SUTURO_MOTION_MAIN_MOTION_MAIN_NODE_H

/**
 * Mainclass for registring the node in the ros environment.
 */
class MotionNode {
private:
    // Variables Declaration
    struct Private;
    ros::NodeHandle node_handle;
    actionlib::SimpleActionServer<motion_msgs::MovingCommandAction> action_server;
    ros::Publisher beliefstatePublisherGrasp;
    ros::Publisher beliefstatePublisherDrop;
    ros::Subscriber perceivedObjectBoundingBoxSubscriber;
    ros::Subscriber jointStateSubscriber;
    moveit::planning_interface::MoveGroup::Plan execution_plan;
    moveit::planning_interface::MoveGroup right_arm_group;
    moveit::planning_interface::MoveGroup left_arm_group;
    moveit::planning_interface::MoveGroup both_arms;
    GroupController group_controller;
    motion_msgs::MovingCommandResult result;
    PlanningSceneController planning_scene_controller;


public:
    /**
     * Constructor.
     * @param nh Node Handle.
     */
    MotionNode(const ros::NodeHandle &nh);

    /**
     * Parses the MovingCommandMessage and executes the requested command.
     * @param goal the goal of the MovingCommandMessage {@link motion_msgs::MovingCommandGoalConstPtr}.
     */
    void executeCommand(const motion_msgs::MovingCommandGoalConstPtr &goal);

    /**
     * Adds the iai kitchen objects to the moveit planning scene.
     * The objects are requested at the knowledge service with the
     * {@link knowledge_msgs::GetFixedKitchenObjects} message.
     *
     * @param res The response of the knowledge service call.
     * @return true/false wether the objects could be added to the planning scene or not.
     */
    bool addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res);

    /**
     * Callback function getting called when a new object was perceived in the scene.
     *
     * @param msg The message containing the name, pose and bounding box of the perceived object.
     */
    void perceivedObjectBoundingBoxCallback(const knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr &msg);

    /**
     * Callback function getting called when a new joint-state message is published.
     *
     * @param msg The message containing the states of the robot-joints.
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
};

/**
 * Starts the motion node.
 */
int start_node(int argc, char **argv);

#endif //SUTURO_MOTION_MAIN_MOTION_MAIN_NODE_H
