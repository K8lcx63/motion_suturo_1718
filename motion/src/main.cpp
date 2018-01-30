#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_msgs/MovingCommandAction.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include "VisualizationMarkerPublisher.h"
#include "MoveGroupController.h"

const int COLOR_SCHEMA_MOTION = 0;
const int COLOR_SCHEMA_KNOWLEDGE = 1;

class Main {
private:
    ros::NodeHandle node_handle;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup right_arm_group;
    moveit::planning_interface::MoveGroup left_arm_group;
    moveit::planning_interface::MoveGroup both_arms;
    geometry_msgs::Pose target_pose1;
    actionlib::SimpleActionServer<motion_msgs::MovingCommandAction> action_server;
    moveit_msgs::MoveItErrorCodes error_code;
    motion_msgs::MovingCommandResult result;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    MoveGroupController moveGroupController;
    VisualizationMarkerPublisher visualizationMarkerPublisher;

public:
    Main(const ros::NodeHandle &nh) :
            node_handle(nh),
            right_arm_group("right_arm"),
            left_arm_group("left_arm"),
            both_arms("arms"),
            moveGroupController(),
            visualizationMarkerPublisher(),
            action_server(node_handle, "moving", boost::bind(&Main::executeCommand, this, _1), false) {
        ros::Publisher vispub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 0);
        visualizationMarkerPublisher.setVisPub(vispub);
        action_server.start();
    }

    bool addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res) {
        int namesSize = res.names.size();
        int posesSize = res.poses.size();
        int boundingBoxesSize = res.bounding_boxes.size();

        if ((namesSize != posesSize) || (posesSize != boundingBoxesSize) || (namesSize != boundingBoxesSize)) {
            ROS_ERROR("Kitchen objects received from knowledge are inconsistent. - Aborted.");
            return false;
        } else {

            std::vector<moveit_msgs::CollisionObject> kitchenObjects;

            //add objects to collision matrix
            for (int i = 0; i < namesSize; i++) {
                std::string name(res.names[i]);
                geometry_msgs::Pose pose = res.poses[i];
                geometry_msgs::Vector3 boundingBox = res.bounding_boxes[i];

                //TODO: compare to which group's planning frame?
                if (res.frame_id != both_arms.getPlanningFrame()) {
                    geometry_msgs::PoseStamped poseIn;
                    poseIn.header.frame_id = res.frame_id;
                    poseIn.pose = pose;

                    geometry_msgs::PoseStamped poseOut;

                    listener.transformPose(both_arms.getPlanningFrame(), poseIn, poseOut);

                    pose.orientation = poseOut.pose.orientation;
                    pose.position = poseOut.pose.position;
                }

                moveit_msgs::CollisionObject kitchenObject;
                kitchenObject.header.frame_id = both_arms.getPlanningFrame();
                kitchenObject.id = name;

                shape_msgs::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = boundingBox.z;
                primitive.dimensions[1] = boundingBox.x;
                primitive.dimensions[2] = boundingBox.y;

                kitchenObject.primitives.push_back(primitive);
                kitchenObject.primitive_poses.push_back(pose);
                kitchenObject.operation = kitchenObject.ADD;

                kitchenObjects.push_back(kitchenObject);
            }

            planning_scene_interface.addCollisionObjects(kitchenObjects);

            return true;
        }
    }

    void executeCommand(const motion_msgs::MovingCommandGoalConstPtr &goal) {
        geometry_msgs::PointStamped goal_point(goal->point_stamped);
        switch (goal->command) {
            case motion_msgs::MovingCommandGoal::MOVE_STANDARD_POSE :
                ROS_INFO("Starting to move to initial pose.");
                both_arms.setNamedTarget("arms_initial");
                error_code = both_arms.move();
                break;
            case motion_msgs::MovingCommandGoal::MOVE_RIGHT_ARM:
                ROS_INFO("Planning to move right arm to: ");
                error_code = moveGroupController.moveGroupToCoordinates(visualizationMarkerPublisher, right_arm_group, goal_point);
                break;
            case motion_msgs::MovingCommandGoal::MOVE_LEFT_ARM:
                ROS_INFO("Planning to move left arm to: ");
                error_code = moveGroupController.moveGroupToCoordinates(visualizationMarkerPublisher, left_arm_group, goal_point);
                break;
            default:
                ROS_ERROR("Got an unknown command constant. Can't do something. Make sure to call"
                                  " the Service with the right constants from the msg file.");
                result.successful = false;
                action_server.setAborted(result, "UNKNOWN COMMAND - ABORTED");
                return;
        }

        handleErrorAndReturnResult();
    }

    void handleErrorAndReturnResult() {
        switch (error_code.val) {
            case moveit_msgs::MoveItErrorCodes::SUCCESS:
                result.successful = true;
                result.status = motion_msgs::MovingCommandResult::SUCCESS;
                action_server.setSucceeded(result);
                ROS_INFO("\x1B[32mX: Moved successfully to goal.");
                break;
            case moveit_msgs::MoveItErrorCodes::FAILURE:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "FAILURE");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("FAILURE.");
                break;
            case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::OUT_OF_RANGE;
                action_server.setAborted(result, "PLANNING FAILED");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("PLANNING FAILED.");
                break;
            case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "INVALID MOTION PLAN");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("INVALID MOTION PLAN.");
                break;
            case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "MOTION PLAN INVALIDATED BY ENVIRONMENT CHANGE");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("MOTION PLAN INVALIDATED BY ENVIRONMENT CHANGE.");
                break;
            case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "CONTROL FAILED");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("CONTROL FAILED.");
                break;
            case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "UNABLE TO AQUIRE SENSOR DATA");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("UNABLE TO AQUIRE SENSOR DATA.");
                break;
            case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "TIMED OUT");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("TIMED OUT.");
                break;
            case moveit_msgs::MoveItErrorCodes::PREEMPTED:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "PREEMPTED");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("PREEMPTED.");
                break;
            case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::COLLISION;
                action_server.setAborted(result, "START STATE IN COLLISION");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("START STATE IN COLLISION.");
                break;
            case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "START STATE VIOLATES PATH CONSTRAINTS");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("START STATE VIOLATES PATH CONSTRAINTS.");
                break;
            case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::COLLISION;
                action_server.setAborted(result, "GOAL IN COLLISION");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("GOAL IN COLLISION.");
                break;
            case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "GOAL VIOLATES PATH CONSTRAINTS");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("GOAL VIOLATES PATH CONSTRAINTS.");
                break;
            case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "GOAL CONSTRAINTS VIOLATED");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("GOAL CONSTRAINTS VIOLATED.");
                break;
            case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "INVALID GROUP NAME");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("INVALID GROUP NAME.");
                break;
            case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "INVALID GOAL CONSTRAINTS");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("INVALID GOAL CONSTRAINTS.");
                break;
            case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "INVALID ROBOT STATE");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("INVALID ROBOT STATE.");
                break;
            case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "INVALID LINK NAME");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("INVALID LINK NAME.");
                break;
            case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "INVALID OBJECT NAME");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("INVALID OBJECT NAME.");
                break;
            case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "FRAME TRANSFORM FAILURE");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("FRAME TRANSFORM FAILURE.");
                break;
            case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "COLLISION CHECKING UNAVAILABLE");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("COLLISION CHECKING UNAVAILABLE.");
                break;
            case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "ROBOT STATE STALE");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("ROBOT STATE STALE.");
                break;
            case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "SENSOR INFO STALE");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("SENSOR INFO STALE.");
                break;
            case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "NO IK SOLUTION");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("NO IK SOLUTION.");
                break;
            default:
                result.successful = false;
                result.status = motion_msgs::MovingCommandResult::UNMANAGEBLE_ERROR;
                action_server.setAborted(result, "UNKNOWN ERROR");
                ROS_ERROR("Movement aborted. Errorcode: ");
                ROS_ERROR("UNKNOWN ERROR.");
                break;

        }
    }

    moveit_msgs::MoveItErrorCodes
    moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group,
                           const geometry_msgs::PointStamped &goal_point) {
        geometry_msgs::PointStamped point;

        //VisualizationMarkerPublisher::publishVisualizationMarker(vis_pub, goal_point, VisualizationMarkerPublisher::TYPE_KNOWLEDGE);
        geometry_msgs::PointStamped tempPoint;
        tempPoint.header = goal_point.header;
        tempPoint.point.x = goal_point.point.x;
        tempPoint.point.y = goal_point.point.y;
        tempPoint.point.z = goal_point.point.z;
        ROS_INFO("Transforming Point from %s to %s", goal_point.header.frame_id.c_str(),
                 group.getPlanningFrame().c_str());
        listener.transformPoint(group.getPlanningFrame(), tempPoint, point);
        ROS_INFO("----Transformed point----");
        ROS_INFO("x %g", point.point.x);
        ROS_INFO("y %g", point.point.y);
        ROS_INFO("z %g", point.point.z);

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = point.header.frame_id;
        poseStamped.pose.position.x = point.point.x;
        poseStamped.pose.position.y = point.point.y;
        poseStamped.pose.position.z = point.point.z;
        poseStamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);
        group.setPoseTarget(poseStamped);
        group.setGoalTolerance(0.05);

        //group.setPositionTarget(point.point.x, point.point.y, point.point.z);
        //publishVisualizationMarker(point, COLOR_SCHEMA_MOTION);

        return group.move();
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_main");
    ros::NodeHandle node_handle;

    Main main(node_handle);

    //add kitchen models to collision detection matrix
    ros::ServiceClient kitchenObjectsClient = node_handle.serviceClient<knowledge_msgs::GetFixedKitchenObjects>(
            "/kitchen_model_service/get_fixed_kitchen_objects");


    knowledge_msgs::GetFixedKitchenObjects srv;

    if (kitchenObjectsClient.call(srv)) {
        ROS_INFO("Received kitchen objects from knowledge service, start to add objects to collision matrix.");
        if (main.addKitchenCollisionObjects(srv.response)) {
            ROS_INFO("Successfully added kitchen objects to collision matrix.");
        } else {
            ROS_ERROR(
                    "Could not add kitchen to collision matrix, because the data received from knowledge service was not correct.");
            return 1;
        }
    } else {
        ROS_ERROR("Could not add kitchen to collision matrix, because knowledge service is not available.");
        return 1;
    }

    ros::spin();

    return 0;
}


