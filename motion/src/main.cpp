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

const int COLOR_SCHEMA_MOTION = 0;
const int COLOR_SCHEMA_KNOWLEDGE= 1;

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

    bool addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res){
        int namesSize = res.names.size();
        int posesSize = res.poses.size();
        int boundingBoxesSize = res.bounding_boxes.size();

        if((namesSize != posesSize) || (posesSize != boundingBoxesSize) || (namesSize != boundingBoxesSize)){
            ROS_ERROR("Kitchen objects received from knowledge are inconsistent. - Aborted.");
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

    /**
     * Publishes a visualization marker.
     *
     * @param point of the visualization marker as pointStamped.
     * @param color_schema ColorSchema, 0 = Red Point, 1 = Yellow Point.
     */
    void publishVisualizationMarker(geometry_msgs::PointStamped point, int color_schema){
        visualization_msgs::Marker marker;
        marker.header.frame_id = point.header.frame_id;
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.point.x;
        marker.pose.position.y = point.point.y;
        marker.pose.position.z = point.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 0.7;
        if (color_schema == COLOR_SCHEMA_MOTION) {
            marker.ns = "motion";
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (color_schema == COLOR_SCHEMA_KNOWLEDGE) {
            marker.ns = "knowledge";
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        vis_pub.publish(marker);
    }

    moveit_msgs::MoveItErrorCodes
    moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &goal_point, bool transform) {
        geometry_msgs::PointStamped point;
        if (transform) {
            publishVisualizationMarker(goal_point, COLOR_SCHEMA_KNOWLEDGE);
            geometry_msgs::PointStamped tempPoint;
            tempPoint.header = goal_point.header;
            tempPoint.point.x = goal_point.point.x;
            tempPoint.point.y = goal_point.point.y;
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
        /*geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = point.header.frame_id;
        poseStamped.pose.position.x = point.point.x;
        poseStamped.pose.position.y = point.point.y;
        poseStamped.pose.position.z = point.point.z;
        poseStamped.pose.orientation.x = 0;
        poseStamped.pose.orientation.y = 0;
        poseStamped.pose.orientation.z = 0;
        poseStamped.pose.orientation.w = 1.0;
        group.setPoseTarget(poseStamped);*/
        group.setPositionTarget(point.point.x, point.point.y, point.point.z);
        publishVisualizationMarker(point, COLOR_SCHEMA_MOTION);
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


