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
#include "factory/MarkerFactory.h"
#include "publisher/MarkerPublisher.h"
#include "moveit/KitchenCollisionObjectService.h"


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
    ros::Publisher vis_pub;

public:
    Main(const ros::NodeHandle &nh, const ros::Publisher &vispub) :
            node_handle(nh),
            right_arm_group("right_arm"),
            left_arm_group("left_arm"),
            both_arms("arms"),
            action_server(node_handle, "moving", boost::bind(&Main::executeCommand, this, _1), false) {
            vis_pub = vispub;
            action_server.start();
    }

    bool addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res) {
        KitchenCollisionObjectService serv;
        return serv.addKitchenCollisionObjects(planning_scene_interface, listener, res);
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
                error_code = moveGroupToCoordinates(right_arm_group, goal_point, true);
                break;
            case motion_msgs::MovingCommandGoal::MOVE_LEFT_ARM:
                ROS_INFO("Planning to move left arm to: ");
                error_code = moveGroupToCoordinates(left_arm_group, goal_point, true);
                break;
            case 4:
                error_code = moveGroupToCoordinates(right_arm_group, goal_point, false);
                break;
            case 5:
                error_code = moveGroupToCoordinates(left_arm_group, goal_point, false);
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
        if (error_code.val == error_code.SUCCESS) {
            result.successful = true;
            action_server.setSucceeded(result);
            ROS_INFO("\x1B[32mX: Moved successfully to goal.");
        } else {
            result.successful = false;

            std::string error_string;
            std::ostringstream convert;

            convert << error_code.val;

            error_string = convert.str();

            action_server.setAborted(result, error_string);
            ROS_ERROR("Movement aborted. Errorcode: ");
            ROS_ERROR("%s", error_string.c_str());
        }
    }


    moveit_msgs::MoveItErrorCodes
    moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &goal_point, bool transform) {
        geometry_msgs::PointStamped point;
        if (transform) {
            MarkerPublisher::publishVisualizationMarker(vis_pub, goal_point, MarkerPublisher::COLOR_SCHEMA_KNOWLEDGE);
            geometry_msgs::PointStamped tempPoint;
            tempPoint.header = goal_point.header;
            tempPoint.point.x = goal_point.point.y;
            tempPoint.point.y = goal_point.point.x;
            tempPoint.point.z = goal_point.point.z;
            ROS_INFO("Transforming Point from %s to %s", goal_point.header.frame_id.c_str(), group.getPlanningFrame().c_str());
            listener.transformPoint(group.getPlanningFrame(), tempPoint, point);
            ROS_INFO("----Transformed point----");
            ROS_INFO("x %g", point.point.x);
            ROS_INFO("y %g", point.point.y);
            ROS_INFO("z %g", point.point.z);
        } else {
            point = goal_point;
        }
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = point.header.frame_id;
        poseStamped.pose.position.x = point.point.x;
        poseStamped.pose.position.y = point.point.y;
        poseStamped.pose.position.z = point.point.z;
        poseStamped.pose.orientation.x = 0.70717;
        poseStamped.pose.orientation.y = 0;
        poseStamped.pose.orientation.z = 0;
        poseStamped.pose.orientation.w = 0.70717;
        group.setPoseTarget(poseStamped);
        MarkerPublisher::publishVisualizationMarker(vis_pub, point, MarkerPublisher::COLOR_SCHEMA_MOTION);
        group.setGoalTolerance(0.05);
        //group.setPositionTarget(point.point.x, point.point.y, point.point.z);
        return group.move();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_main");
    ros::NodeHandle node_handle;

    ros::Publisher pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    Main main(node_handle, pub);

    //add kitchen models to collision detection matrix
    ros::ServiceClient kitchenObjectsClient = node_handle.serviceClient<knowledge_msgs::GetFixedKitchenObjects>("/kitchen_model_service/get_fixed_kitchen_objects");


    knowledge_msgs::GetFixedKitchenObjects srv;

    if(kitchenObjectsClient.call(srv)){
        ROS_INFO("Received kitchen objects from knowledge service, start to add objects to collision matrix.");
        if(main.addKitchenCollisionObjects(srv.response)){
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


