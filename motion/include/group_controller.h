#ifndef SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
#define SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
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
#include <math.h>
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
    const float LIFTING_AFTER_GRASPING = 0.08f;
    const float FORCE_THRESHOLD = 15;
    const int MAX_ATTEMPTS_TO_GET_IK_SOLUTION = 2;

    const string PATH_TO_GRIPPER_MESH = "package://knowledge_common/meshes/Gripper/Gripper.stl";

    ros::NodeHandle nodeHandle;
    ros::Publisher robotStatePublisher;
    ros::ServiceClient ikServiceClient;
    ros::Publisher beliefstatePublisherGrasp;
    ros::Publisher beliefstatePublisherDrop;
    ros::Subscriber ftSensorSubscriber;

    moveit::planning_interface::MoveGroup::Plan execution_plan;
    PointTransformer point_transformer;
    VisualizationMarker visualizationMarker;
    actionlib::SimpleActionClient<motion_msgs::GripperAction> gripperclient;
    PlanningSceneController planning_scene_controller;
    float forceMagnitude;

    //indicates, whether actually a simulation is launched or the real robot is used.
    bool isSimulation;

    vector<string> getGripperLinks(int gripper);

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

    /** Uses the given {@link moveit::planning_interface::MoveGroup} to poke at the point and with the orientation
     * as given in {@link geometry_msgs::PointStamped}.
     * @param group The group to take. 
     * @param goalPose The point to poke and the orientation the gripper shall have.
     * @param objectLabel the label of the object that get's poked.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the poking action. 
     */ 
    moveit_msgs::MoveItErrorCodes pokeObject(moveit::planning_interface::MoveGroup& group,
                                             const geometry_msgs::PoseStamped& goalPose, const string objectLabel);
 
    /** 
     * Uses the given {@link moveit::planning_interface::MoveGroup} to grasp the object with the given grasp pose in
     * {@link geometry_msgs::PoseStamped}.
     * The {@link ros::Publisher} is for publishing a message to a topic after grasping
     * the object. This is needed for the beliefstate.
     * The string contains the label of the object to grasp.
     * @param group The group to take. 
     * @param objectGraspPoses The poses how the object may be grasped.
     * @param objectLabel The name of the object to grasp.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the grasping action. 
     */ 
    moveit_msgs::MoveItErrorCodes graspObject(moveit::planning_interface::MoveGroup& group,
                                              const geometry_msgs::PoseArray& objectGraspPoses,
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
     * @param objectLabel The name of the object to place.
     * @return {@link moveit_msgs::MoveItErrorCodes} with the result of the droping action.
     */
    moveit_msgs::MoveItErrorCodes placeObject(moveit::planning_interface::MoveGroup& group,
                                              const geometry_msgs::PoseStamped& object_drop_pose, std::string objectLabel);

    /**
     * Takes the request and tries up to two times to find an collision free ik solution for this request. If any solution is found,
     * 'ikResponse' is set to the resulting solution and true is returned. Otherwise, false is returned.
     *
     * @param ikRequest the ik request to solve, containing the goal pose for the wrist.
     * @param ikResponse the solution, if any found.
     * @return true, if a solution was found, else false is returned.
     */
    bool getIkSolution(const moveit_msgs::GetPositionIK::Request &ikRequest, moveit_msgs::GetPositionIK::Response &ikResponse);

    /**
     * Visualizes the ik solution in rviz and returns the robot state of the ik solution.
     *
     * @param solution the ik solution to visualize.
     * @return the state of the robot as given in the ik solution.
     */
    moveit::core::RobotStatePtr visualizeIkSolution(const moveit_msgs::GetPositionIK::Response &solution);

    /**
     * Ranks the given ik solutions by two criteria:
     *      - distance of goal robot state to next collision (factor 0.65)
     *      - distance of current robot state to goal robot state (factor 0.35)
     *
     * The two lists are sorted in the order they get ranked, starting at index 0 with the best rated goal pose.
     *
     * @param indices the indices of the grasp poses in the original list of all grasp poses, the ik solution in the second
     *                list at the same index belongs to.
     * @param solutions the ik solution for the grasp pose at the index given in the first list.
     * @param currentRobotState the current state of the robot
     * @param objectLabel the label of the object currently shall be grasped.
     * @param gripper the gripper to grasp the object with. Constants for the grippers are defined in motion_msgs::GripperGoal.
     */
    void rankGraspPoses(vector<int> &indices, vector<moveit_msgs::GetPositionIK::Response> &solutions, const robot_state::RobotStatePtr &currentState,
                        const string objectLabel, int gripper);

    /**
     * Calculates and returns the distance between the robot state given in currentState and solutionState.
     *
     * @param currentState the current state of the robot.
     * @param robotInitialState the initial kinematic model of the robot.
     * @param solutionState the state to which the distance shall be calculated.
     * @return the distance between the two states.
     */
    double getStateDistance(const robot_state::RobotStatePtr &currentState, robot_state::RobotState &robotInitialState,
                                             const moveit_msgs::RobotState &solutionState);

    /**
     * Allows object to collide with all objects it actually is colliding with.
     * Used after closing the gripper when grasping an object.
     * Because then, the gripper and the table, the object actually is placed on, collide with the grasped object and the
     * robot can not move anymore, because his start state is in collision.
     * This collisions get allowed for a short moment. Then, the grasped object is lifted for some cm to get it out
     * of the colliding state. At that point of time, the function 'allowCollisionForGrasping' is called, what causes
     * the collision allowed in this function getting forbidden again. Then, only the collision with the gripper holding
     * the object is allowed.
     *
     * @param objectLabel the object to grasp. Allow collision for this with colliding objects, like table
     *                    and gripper. Allowing collision with everything else than the gripper gets disabled again after
     *                    a short time.
     * @param gripper the gripper to allow collision with. Constants for the grippers are defined in motion_msgs::GripperGoal.
     * @return true if the collision was successfully allowed
     */
    bool allowCollisionWithCollidingObjects(const string objectLabel, int gripper);

    /**
     * Allows the collision for the given object with the given gripper.
     *
     * @param objectName the name of the object which is allowed to collide with the gripper.
     * @param gripper the gripper the object is allowed to collide with. Constants for the grippers are defined in motion_msgs::GripperGoal.
     * @return true if the collision was successfully allowed
     */
    bool allowCollisionForGrasping(const string objectName, int gripper);

    /**
     * Checks if the object was successfully grasped.
     * @param gripperNum the number of the gripper to check.
     * @return true if the gripper has something in the gripper given by gripperNum. Constants for the grippers are defined in motion_msgs::GripperGoal.
     */
    bool checkIfObjectGraspedSuccessfully(int gripperNum);

    /** 
     * Opens the gripper given by gripperName. 
     * @param gripperNum the number of the gripper to close. Constants for the grippers are defined in motion_msgs::GripperGoal.
     */
    void openGripper(int gripperNum);

    /**
     * Closes the gripper given by gripperName.
     * @param gripperNum the number of the gripper to close. Constants for the grippers are defined in motion_msgs::GripperGoal.
     */
    void closeGripper(int gripperNum, double& effort);

    /**
     * Callback function for topic on which the force torque sensor data get published.
     * Calculates the magnitude of the force-vector and saves it in a global variable.
     *
     * @param msg the message containing the data about the force and the torque values of the sensor.
     */
    void ftSensorCallback (const geometry_msgs::WrenchStamped::ConstPtr &msg);
};


#endif //SUTURO_MOTION_MAIN_GROUP_CONTROLLER_H
