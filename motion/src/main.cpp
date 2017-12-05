#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_msgs/MovingCommandAction.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include <knowledge_msgs/GetFixedKitchenObjects.h>


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

public:
    Main(const ros::NodeHandle &nh) :
            node_handle(nh),
            right_arm_group("right_arm"),
            left_arm_group("left_arm"),
            both_arms("arms"),
            action_server(node_handle, "moving", boost::bind(&Main::executeCommand, this, _1), false) {
            action_server.start();
    }

    bool addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res){
        int namesSize = res.names.size();
        int posesSize = res.poses.size();
        int boundingBoxesSize = res.bounding_boxes.size();

        if((namesSize != posesSize) || (posesSize != boundingBoxesSize) || (namesSize != boundingBoxesSize)){
            ROS_ERROR("Data about kitchen objects received from knowledge are inconsistent.");
            return false;
        }else{

            std::vector<moveit_msgs::CollisionObject> kitchenObjects;

            //add objects to collision matrix
            for(int i = 0; i < namesSize; i++){
                std::string name(res.names[i]);
                geometry_msgs::Pose pose = res.poses[i];
                geometry_msgs::Vector3 boundingBox = res.bounding_boxes[i];

                //TODO: compare to which group's planning frame?
                if(res.frame_id != both_arms.getPlanningFrame()){
                    geometry_msgs::PoseStamped poseIn;
                    poseIn.header.frame_id = res.frame_id;
                    poseIn.pose = pose;

                    geometry_msgs::PoseStamped poseOut;
/*
                    std::vector<std::string> frames;
                    listener.getFrameStrings(frames);

                    for(int i = 0; i < frames.size(); i++){
                        ROS_INFO("%s", frames[i].c_str());

                    }*/


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
                ROS_INFO("Moving to initial pose.");
                both_arms.setNamedTarget("arms_initial");
                error_code = both_arms.move();
                break;
            case motion_msgs::MovingCommandGoal::MOVE_RIGHT_ARM:
                ROS_INFO("Moving right arm to goal.");
                error_code = moveGroupToCoordinates(right_arm_group, goal_point);
                break;
            case motion_msgs::MovingCommandGoal::MOVE_LEFT_ARM:
                ROS_INFO("Moving left arm to goal.");
                error_code = moveGroupToCoordinates(left_arm_group, goal_point);
                break;
            default:
                ROS_ERROR("COMMAND UNKNOWN");
                result.successful = false;
                action_server.setAborted(result, "UNKNOWN COMMAND. ABORTED.");
                return;
        }

        handleErrorAndReturnResult();
    }

    void handleErrorAndReturnResult() {
        if (error_code.val == error_code.SUCCESS) {
            result.successful = true;
            action_server.setSucceeded(result);
            ROS_INFO("MOVE SUCCESSFUL");
        } else {
            result.successful = false;

            std::string error_string;
            std::ostringstream convert;

            convert << error_code.val;

            error_string = convert.str();

            action_server.setAborted(result, error_string);
            ROS_WARN("%s", error_string.c_str());
        }
    }

    moveit_msgs::MoveItErrorCodes
    moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &goal_point) {
        geometry_msgs::PointStamped point;
        listener.transformPoint(group.getPlanningFrame(), goal_point, point);
        //geometry_msgs::PoseStamped poseStamped;
        //poseStamped.pose.position.x = goal_point.point.x;
        //poseStamped.pose.position.y = goal_point.point.y;
        //poseStamped.pose.position.z = goal_point.point.z;
        //poseStamped.pose.orientation.w = 1.0;
        //group.setPoseTarget(poseStamped);
        group.setPositionTarget(point.point.x, point.point.y, point.point.z);
        return group.move();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_main");
    ros::NodeHandle node_handle;

    Main main(node_handle);

    //add kitchen models to collision detection matrix
    ros::ServiceClient kitchenObjectsClient = node_handle.serviceClient<knowledge_msgs::GetFixedKitchenObjects>("/kitchen_model_service/get_fixed_kitchen_objects");
    knowledge_msgs::GetFixedKitchenObjects srv;

    if(kitchenObjectsClient.call(srv)){
        ROS_INFO("Received kitchen objects from knowledge service, start to add objects to collision matrix.");
        if(main.addKitchenCollisionObjects(srv.response)){
            ROS_INFO("Successfully added kitchen objects to collision matrix.");
        }else{
            ROS_ERROR("Could not add kitchen to collision matrix, because the data received from knowledge service were not correct.");
            return 1;
        }
    }else{
        ROS_ERROR("Could not add kitchen to collision matrix, because knowledge service is not available.");
        return 1;
    }

    ros::spin();

    return 0;
}


