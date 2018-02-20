#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_msgs/MovingCommandAction.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <math.h>

const int COLOR_SCHEMA_MOTION = 0;
const int COLOR_SCHEMA_KNOWLEDGE= 1;
const int COLOR_SCHEMA_VISION = 2;
const float GRIPPER_LENGTH_RIGHT = 0.15f;
const float GRIPPER_LENGTH_LEFT = 0.18f;

class Main {
private:
    ros::NodeHandle node_handle;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup::Plan execution_plan;
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

        if (goal->command != motion_msgs::MovingCommandGoal::MOVE_STANDARD_POSE
            && (goal_point.header.frame_id == "" || !listener.frameExists(goal_point.header.frame_id))) {
            ROS_ERROR("Correct reference frame has to be declared for goal.");
            result.successful = false;
            action_server.setAborted(result, "NO REFERENCE FRAME DECLARED.");
            return;
        }

        switch (goal->command) {
            case motion_msgs::MovingCommandGoal::MOVE_STANDARD_POSE :
                ROS_INFO("Starting to move to initial pose.");
                both_arms.setNamedTarget("arms_initial");

                error_code = both_arms.plan(execution_plan);

                if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                    error_code = both_arms.move();
                }

                break;
            case motion_msgs::MovingCommandGoal::MOVE_RIGHT_ARM:
                ROS_INFO("Planning to move right arm to: ");
                moveGroupToCoordinates(right_arm_group, goal_point);
                break;
            case motion_msgs::MovingCommandGoal::MOVE_LEFT_ARM:
                ROS_INFO("Planning to move left arm to: ");
                moveGroupToCoordinates(left_arm_group, goal_point);
                break;
            case motion_msgs::MovingCommandGoal::MOVE_RIGHT_GRIPPER:
                ROS_INFO("Planning to move right gripper to: ");
                moveEndEffectorToGoal(right_arm_group, goal_point);
                break;
            case motion_msgs::MovingCommandGoal::MOVE_LEFT_GRIPPER:
                ROS_INFO("Planning to move left gripper to: ");
                moveEndEffectorToGoal(left_arm_group, goal_point);
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

    /**
     * Publishes a visualization marker.
     *
     * @param point of the visualization marker as pointStamped.
     * @param color_schema ColorSchema, 0 = Red Point, 1 = Yellow Point.
     */
    void publishVisualizationMarker(geometry_msgs::PointStamped point, int color_schema) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_footprint";//point.header.frame_id;
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
        } else if (color_schema == COLOR_SCHEMA_VISION) {
            marker.ns = "vision";
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        vis_pub.publish(marker);
    }

    void
    moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group,
                           const geometry_msgs::PointStamped &goal_point) {
        geometry_msgs::PointStamped point;

        publishVisualizationMarker(goal_point, COLOR_SCHEMA_KNOWLEDGE);
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
        publishVisualizationMarker(point, COLOR_SCHEMA_MOTION);


        error_code = group.plan(execution_plan);


        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            error_code = group.move();
        }

    }

    void
    moveEndEffectorToGoal(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &goal_point) {
        geometry_msgs::PointStamped toVisualize;

        listener.transformPoint("base_footprint", goal_point, toVisualize);
        publishVisualizationMarker(toVisualize, COLOR_SCHEMA_MOTION);
        /*
         * First calculate pose to move endeffector to frontdirection of object
         */

        //plan to move endeffector in front of object
        geometry_msgs::PointStamped frontDirectionOfObject;
        listener.transformPoint("base_footprint", goal_point, frontDirectionOfObject);

        //calculate position in front of object and two-third of the way to the robot
        frontDirectionOfObject.point.x = frontDirectionOfObject.point.x * 3/4;
        publishVisualizationMarker(frontDirectionOfObject, COLOR_SCHEMA_KNOWLEDGE);

        //transform to PoseStamped and set orientation
        geometry_msgs::PoseStamped frontDirectionOfObjectPose;
        frontDirectionOfObjectPose.header.frame_id = frontDirectionOfObject.header.frame_id;
        frontDirectionOfObjectPose.pose.position = frontDirectionOfObject.point;
        //frontDirectionOfObjectPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);
        frontDirectionOfObjectPose.pose.orientation.x = 0;
        frontDirectionOfObjectPose.pose.orientation.y = 0;
        frontDirectionOfObjectPose.pose.orientation.z = 0;
        frontDirectionOfObjectPose.pose.orientation.w = 1;

        //transform to group's planning frame
        geometry_msgs::PoseStamped goalFrontDirectionOfObjectPose;
        listener.transformPose(group.getPlanningFrame(), frontDirectionOfObjectPose, goalFrontDirectionOfObjectPose);

        group.setPoseTarget(goalFrontDirectionOfObjectPose);
        group.setGoalTolerance(0.015);
        error_code = group.plan(execution_plan);

        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

            robot_trajectory::RobotTrajectory trajectory(kinematic_model, group.getName());
            trajectory.setRobotTrajectoryMsg(*(group.getCurrentState()), execution_plan.trajectory_);

            moveit::core::RobotState robotState = trajectory.getLastWayPoint();
            Eigen::Affine3d eef_transform = robotState.getGlobalLinkTransform(group.getEndEffectorLink());

            geometry_msgs::Pose inFrontOfObjectEndEffectorPose;
            tf::poseEigenToMsg(eef_transform, inFrontOfObjectEndEffectorPose);

            if(group.getName() == "right_arm_group"){
                inFrontOfObjectEndEffectorPose.position.x -= GRIPPER_LENGTH_RIGHT;
            }else{
                inFrontOfObjectEndEffectorPose.position.x -= GRIPPER_LENGTH_LEFT;
            }

            geometry_msgs::PoseStamped inFrontOfObjectEndEffectorPoseStamped;
            inFrontOfObjectEndEffectorPoseStamped.pose = inFrontOfObjectEndEffectorPose;
            inFrontOfObjectEndEffectorPoseStamped.header.frame_id = "map";

            geometry_msgs::PointStamped oldPoint;
            geometry_msgs::PointStamped newPoint;
            oldPoint.header.frame_id = inFrontOfObjectEndEffectorPoseStamped.header.frame_id;
            oldPoint.point = inFrontOfObjectEndEffectorPoseStamped.pose.position;
            listener.transformPoint("base_footprint", oldPoint, newPoint);

            //set new goal for movegroup and plan again with new goal
            group.setPoseTarget(inFrontOfObjectEndEffectorPoseStamped);
            group.setGoalTolerance(0.025);

            error_code = group.plan(execution_plan);

            //if plan succeeded, move to goal
            if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                error_code = group.move();



                /*
                * If first movement was successful, calculate path to move endeffector to the object
                */

                //plan to move endeffector to object
                geometry_msgs::PointStamped point;

                ROS_INFO("Transforming Point from %s to %s", goal_point.header.frame_id.c_str(),
                         group.getPlanningFrame().c_str());
                listener.transformPoint(group.getPlanningFrame(), goal_point, point);

                geometry_msgs::PoseStamped objectPosition;
                objectPosition.header.frame_id = point.header.frame_id;
                objectPosition.pose.position.x = point.point.x;
                objectPosition.pose.position.y = point.point.y;
                objectPosition.pose.position.z = point.point.z;
                //objectPosition.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);
                objectPosition.pose.orientation.x = 0;
                objectPosition.pose.orientation.y = 0;
                objectPosition.pose.orientation.z = 0;
                objectPosition.pose.orientation.w = 1;

                //set target for movegroup instance
                group.setPoseTarget(objectPosition);
                group.setGoalTolerance(0.025);

                error_code = group.plan(execution_plan);


                //if planning to move arm to the pose, recalculate last waypoint so that the endeffector is moved to
                //object instead of the arm
                if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

                    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
                    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

                    robot_trajectory::RobotTrajectory trajectory(kinematic_model, group.getName());
                    trajectory.setRobotTrajectoryMsg(*(group.getCurrentState()), execution_plan.trajectory_);

                    moveit::core::RobotState robotState = trajectory.getLastWayPoint();
                    Eigen::Affine3d eef_transform = robotState.getGlobalLinkTransform(group.getEndEffectorLink());

                    geometry_msgs::Pose objectEndEffectorPose;
                    tf::poseEigenToMsg(eef_transform, objectEndEffectorPose);

                    geometry_msgs::PoseStamped objectEndEffectorPoseStamped;
                    objectEndEffectorPoseStamped.pose = objectEndEffectorPose;
                    objectEndEffectorPoseStamped.header.frame_id = "map";

                    geometry_msgs::PoseStamped objectEndEffectorPoseStampedNewHeight;
                    listener.transformPose("base_footprint", objectEndEffectorPoseStamped, objectEndEffectorPoseStampedNewHeight);

                    if(group.getName() == "right_arm_group"){
                        objectEndEffectorPoseStampedNewHeight.pose.position.x += GRIPPER_LENGTH_RIGHT * 2/3;
                    }else{
                        objectEndEffectorPoseStampedNewHeight.pose.position.x += GRIPPER_LENGTH_LEFT * 2/3;
                    }
                    objectEndEffectorPoseStampedNewHeight.pose.position.z += 0.04;

                    //set new goal for movegroup and plan again with new goal
                    group.setPoseTarget(objectEndEffectorPoseStampedNewHeight);
                    group.setGoalTolerance(0.015);

                    geometry_msgs::PointStamped markerPoint;
                    markerPoint.header = objectEndEffectorPoseStampedNewHeight.header;
                    markerPoint.point = objectEndEffectorPoseStampedNewHeight.pose.position;
                    publishVisualizationMarker(markerPoint, COLOR_SCHEMA_VISION);

                    //von frontdirectionofobject trajectorie richtung markerpoint mit orientierung
                    //setposereferenceframe auf base footprint
                    //computecartesianpath

                    geometry_msgs::Vector3 directionVector;
                    directionVector.x = markerPoint.point.x - frontDirectionOfObject.point.x;
                    directionVector.y = markerPoint.point.y - frontDirectionOfObject.point.y;
                    directionVector.z = markerPoint.point.z - frontDirectionOfObject.point.z;

                    visualization_msgs::Marker marker2;
                    marker2.id = 5;
                    marker2.header.stamp = ros::Time::now();
                    marker2.header.frame_id = "/base_footprint";//point.header.frame_id;
                    marker2.type = visualization_msgs::Marker::ARROW;
                    marker2.action = visualization_msgs::Marker::ADD;
                    marker2.points.push_back(frontDirectionOfObject.point);
                    geometry_msgs::Point end;
                    end.x = frontDirectionOfObject.point.x + directionVector.x;
                    end.y = frontDirectionOfObject.point.y + directionVector.y;
                    end.z = frontDirectionOfObject.point.z + directionVector.z;
                    marker2.points.push_back(end);
                    marker2.pose.orientation.x = 0.0;
                    marker2.pose.orientation.y = 0.0;
                    marker2.pose.orientation.z = 0.0;
                    marker2.pose.orientation.w = 1.0;
                    marker2.scale.x = 0.05;
                    marker2.scale.y = 0.05;
                    marker2.scale.z = 0.05;
                    marker2.color.a = 0.7;
                    marker2.ns = "waypoints";
                    marker2.color.r = 0.2;
                    marker2.color.g = 0.2;
                    marker2.color.b = 0.5;
                    //vis_pub.publish(marker2);

                    /*ROS_INFO("STARTX:  %f", frontDirectionOfObject.point.x);
                    ROS_INFO("STARTY:  %f", frontDirectionOfObject.point.y);
                    ROS_INFO("STARTZ:  %f", frontDirectionOfObject.point.z);
                    ROS_INFO("ENDX:  %f", end.x);
                    ROS_INFO("ENDY:  %f", end.y);
                    ROS_INFO("ENDZ:  %f", end.z);*/

                    float distance = sqrt(pow(directionVector.x, 2.0) + pow(directionVector.y, 2.0)
                            + pow(directionVector.z, 2.0));

                    //sample 10 points for trajectory
                    geometry_msgs::Vector3 directionStep;
                    directionStep.x = directionVector.x / 10.0;
                    directionStep.y = directionVector.y / 10.0;
                    directionStep.z = directionVector.z / 10.0;

                    std::vector<geometry_msgs::Pose> waypoints;


                    visualization_msgs::Marker marker;
                    marker.id = 0;
                    marker.header.stamp = ros::Time::now();
                    marker.header.frame_id = "/base_footprint";//point.header.frame_id;
                    marker.type = visualization_msgs::Marker::SPHERE_LIST;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.03;
                    marker.scale.y = 0.03;
                    marker.scale.z = 0.03;
                    marker.color.a = 0.7;
                    marker.ns = "waypoints";
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.0;
                    for(double i = 1; i <= 10; i++){
                        geometry_msgs::Pose waypoint;
                        geometry_msgs::Vector3 step;

                        step.x = directionVector.x * (i/10.0);
                        step.y = directionVector.y * (i/10.0);
                        step.z = directionVector.z * (i/10.0);

                        waypoint.position.x = frontDirectionOfObject.point.x + step.x;
                        waypoint.position.y = frontDirectionOfObject.point.y + step.y;
                        waypoint.position.z = frontDirectionOfObject.point.z + step.z;
                        //waypoint.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);
                        waypoint.orientation.x = 0;
                        waypoint.orientation.y = 0;
                        waypoint.orientation.z = 0;
                        waypoint.orientation.w = 1;


                        waypoints.push_back(waypoint);

                        geometry_msgs::Point vis_marker;
                        vis_marker.x = waypoint.position.x;
                        vis_marker.y = waypoint.position.y;
                        vis_marker.z = waypoint.position.z;
                       // vis_marker.header.frame_id = "/base_footprint";
                        //publishVisualizationMarker(vis_marker, COLOR_SCHEMA_MOTION);

                        ROS_INFO(" X:  %f", waypoint.position.x);
                        ROS_INFO(" Y:  %f", waypoint.position.y);
                        ROS_INFO(" Z:  %f", waypoint.position.z);

                        marker.points.push_back(vis_marker);
                    }

                    vis_pub.publish(marker);

                    group.setPoseReferenceFrame("base_footprint");
                    moveit_msgs::RobotTrajectory robotTrajectory;
                    double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, robotTrajectory);

                    if(fraction != -1){
                        robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
                        rt.setRobotTrajectoryMsg(*group.getCurrentState(), robotTrajectory);
                        /*trajectory_processing::IterativeParabolicTimeParameterization iptp;
                        success = iptp.computeTimeStamps(rt);
                        ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");*/

                        rt.getRobotTrajectoryMsg(robotTrajectory);



                        execution_plan.trajectory_ = robotTrajectory;
                        ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);
                        sleep(5.0);
                        group.execute(execution_plan);



                    }
                    /*
                    error_code = group.plan(execution_plan);

                    //if plan succeeded, move to goal
                    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                        error_code = group.move();*/

                }
            }

        }


    }
};

    int main(int argc, char **argv) {
        ros::init(argc, argv, "motion_main");
        ros::NodeHandle node_handle;

        ros::Publisher pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        Main main(node_handle, pub);

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


