#include <ros/init.h>
#include "../include/node/motion_main_node.h"

int start_node(int argc, char **argv) {
    ros::init(argc, argv, "suturo_motion_main");
    ros::NodeHandle nh;
    MotionNode motionNode(nh);
    //add kitchen models to collision detection matrix
    ros::ServiceClient kitchenObjectsClient = nh.serviceClient<knowledge_msgs::GetFixedKitchenObjects>("/kitchen_model_service/get_fixed_kitchen_objects");
    knowledge_msgs::GetFixedKitchenObjects srv;
    if(kitchenObjectsClient.call(srv)){
        ROS_INFO("Received kitchen objects from knowledge service, start to add objects to collision matrix.");
        if(motionNode.addKitchenCollisionObjects(srv.response)){
            ROS_INFO("Successfully added kitchen objects to collision matrix.");
        }else{
            ROS_ERROR("Could not add kitchen to collision matrix, because the data received from knowledge service was not correct.");
            return 1;
        }
    }else{
        ROS_ERROR("Could not add kitchen to collision matrix, because knowledge service is not available.");
        return 1;
    }

    ros::spin();

    return 0;
}

MotionNode::MotionNode(const ros::NodeHandle &nh) :
node_handle(nh),
right_arm_group("right_arm"),
left_arm_group("left_arm"),
both_arms("arms"),
action_server(node_handle, "moving", boost::bind(&MotionNode::executeCommand, this, _1), false) {
    action_server.start();
}

struct MotionNode::Private {
    static void handleErrorAndReturnResult(moveit_msgs::MoveItErrorCodes& error_code, actionlib::SimpleActionServer<motion_msgs::MovingCommandAction>& action_server, motion_msgs::MovingCommandResult& result){
        switch(error_code.val){
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
                result.status = motion_msgs::MovingCommandResult::COLLISION;
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
};

void MotionNode::executeCommand(const motion_msgs::MovingCommandGoalConstPtr &goal) {
    moveit_msgs::MoveItErrorCodes error_code;
    geometry_msgs::PointStamped goal_point(goal->point_stamped);
    switch (goal->command) {
        case motion_msgs::MovingCommandGoal::MOVE_STANDARD_POSE :
            ROS_INFO("Starting to move to initial pose.");
            error_code = group_controller.moveArmsToInitial(both_arms);
            break;
        case motion_msgs::MovingCommandGoal::MOVE_RIGHT_ARM:
            ROS_INFO("Planning to move right arm to: ");
            error_code = group_controller.moveGroupToCoordinates(right_arm_group, goal_point);
            break;
        case motion_msgs::MovingCommandGoal::MOVE_LEFT_ARM:
            ROS_INFO("Planning to move left arm to: ");
            error_code = group_controller.moveGroupToCoordinates(left_arm_group, goal_point);
            break;
        case motion_msgs::MovingCommandGoal::MOVE_LEFT_GRIPPER:
            ROS_INFO("Planning to move left gripper to: ");
            error_code = group_controller.moveEndEffectorToGoal(left_arm_group, goal_point);
            break;
        case motion_msgs::MovingCommandGoal::MOVE_RIGHT_GRIPPER:
            ROS_INFO("Planning to move right gripper to: ");
            error_code = group_controller.moveEndEffectorToGoal(right_arm_group, goal_point);
            break;
        default:
            ROS_ERROR("Got an unknown command constant. Can't do something. Make sure to call"
                              " the Service with the right constants from the msg file.");
            result.successful = false;
            action_server.setAborted(result, "UNKNOWN COMMAND - ABORTED");
            return;
    }
    Private::handleErrorAndReturnResult(error_code, action_server, result);
}

bool MotionNode::addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res) {
    return planning_scene_controller.addKitchenCollisionObjects(res, both_arms.getPlanningFrame(), planning_scene_interface);
}



