#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "../include/movegroup/group_controller.h"
#include "../include/visualization/visualization_marker.h"

moveit_msgs::MoveItErrorCodes
GroupController::moveGroupToCoordinates(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& goal_point) {
    geometry_msgs::PointStamped point = point_transformer.transformPointStamped(group, goal_point);
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = point.header.frame_id;
    poseStamped.pose.position.x = point.point.x;
    poseStamped.pose.position.y = point.point.y;
    poseStamped.pose.position.z = point.point.z;
    poseStamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);
    group.setPoseTarget(poseStamped);
    group.setGoalTolerance(0.05);

    moveit::planning_interface::MoveItErrorCode error_code = group.plan(execution_plan);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        error_code = group.move();
    }
    return error_code;
}

moveit_msgs::MoveItErrorCodes GroupController::moveEndEffectorToGoal(moveit::planning_interface::MoveGroup &group,
                                                                     const geometry_msgs::PointStamped &goal_point) {
    //publishVisualizationMarker(goal_point, COLOR_SCHEMA_MOTION);
    //First calculate pose to move endeffector to frontdirection of object

    //plan to move endeffector in front of object
    geometry_msgs::PointStamped frontDirectionOfObject = point_transformer.transformPointStamped("base_footprint", goal_point);

    //calculate position in front of object and half of the way to the robot
    frontDirectionOfObject.point.x = frontDirectionOfObject.point.x / 2;
    ROS_INFO("X: %g", frontDirectionOfObject.point.x);
    ROS_INFO("Y: %g", frontDirectionOfObject.point.y);
    ROS_INFO("Z: %g", frontDirectionOfObject.point.z);

    //transform to PoseStamped and set orientation
    geometry_msgs::PoseStamped frontDirectionOfObjectPose;
    frontDirectionOfObjectPose.header.frame_id = frontDirectionOfObject.header.frame_id;
    frontDirectionOfObjectPose.pose.position = frontDirectionOfObject.point;
    frontDirectionOfObjectPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);

    //transform to group's planning frame
    geometry_msgs::PoseStamped goalFrontDirectionOfObjectPose = point_transformer.transformPoseStamped(group.getPlanningFrame(), frontDirectionOfObjectPose);

    group.setPoseTarget(goalFrontDirectionOfObjectPose);
    group.setGoalTolerance(0.015);
    moveit::planning_interface::MoveItErrorCode error_code = group.plan(execution_plan);

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

        ROS_INFO("X: %g", inFrontOfObjectEndEffectorPose.position.x);
        ROS_INFO("Y: %g", inFrontOfObjectEndEffectorPose.position.y);
        ROS_INFO("Z: %g", inFrontOfObjectEndEffectorPose.position.z);

        geometry_msgs::PoseStamped inFrontOfObjectEndEffectorPoseStamped;
        inFrontOfObjectEndEffectorPoseStamped.pose = inFrontOfObjectEndEffectorPose;
        inFrontOfObjectEndEffectorPoseStamped.header.frame_id = "map";
        ROS_INFO_STREAM("POSESTAMPED FRAME: " << inFrontOfObjectEndEffectorPoseStamped.header.frame_id);

        geometry_msgs::PointStamped oldPoint;
        oldPoint.header.frame_id = inFrontOfObjectEndEffectorPoseStamped.header.frame_id;
        oldPoint.point = inFrontOfObjectEndEffectorPoseStamped.pose.position;
        geometry_msgs::PointStamped newPoint = point_transformer.transformPointStamped("base_footprint", oldPoint);

        //set new goal for movegroup and plan again with new goal
        group.setPoseTarget(inFrontOfObjectEndEffectorPoseStamped);
        group.setGoalTolerance(0.015);

        error_code = group.plan(execution_plan);

        //if plan succeeded, move to goal
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            error_code = group.move();
        }
        return error_code;
    }

    /*
     * Then calculate pose to move endeffector to the object
     */

    //plan to move endeffector to object
    geometry_msgs::PointStamped point = point_transformer.transformPointStamped(group.getPlanningFrame(), goal_point);

    geometry_msgs::PoseStamped objectPosition;
    objectPosition.header.frame_id = point.header.frame_id;
    objectPosition.pose.position.x = point.point.x;
    objectPosition.pose.position.y = point.point.y;
    objectPosition.pose.position.z = point.point.z;
    objectPosition.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);

    //set target for movegroup instance
    group.setPoseTarget(objectPosition);
    group.setGoalTolerance(0.015);

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

        if(group.getName() == "right_arm_group"){
            objectEndEffectorPose.position.x -= GRIPPER_LENGTH_RIGHT/3;
        }else{
            objectEndEffectorPose.position.x -= GRIPPER_LENGTH_LEFT/3;
        }

        geometry_msgs::PoseStamped objectEndEffectorPoseStamped;
        objectEndEffectorPoseStamped.pose = objectEndEffectorPose;
        objectEndEffectorPoseStamped.header.frame_id = "map";

        geometry_msgs::PoseStamped objectEndEffectorPoseStampedNewHeight = point_transformer.transformPoseStamped("base_footprint", objectEndEffectorPoseStamped);

        objectEndEffectorPoseStampedNewHeight.pose.position.z += 0.03;

        //set new goal for movegroup and plan again with new goal
        group.setPoseTarget(objectEndEffectorPoseStampedNewHeight);
        group.setGoalTolerance(0.015);

        geometry_msgs::PointStamped markerPoint;
        markerPoint.header = objectEndEffectorPoseStampedNewHeight.header;
        markerPoint.point = objectEndEffectorPoseStampedNewHeight.pose.position;

        error_code = group.plan(execution_plan);

        //if plan succeeded, move to goal
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            error_code = group.move();

    }
}

moveit_msgs::MoveItErrorCodes GroupController::moveArmsToInitial(moveit::planning_interface::MoveGroup &group) {
    group.setNamedTarget("arms_initial");

    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);

    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        error_code = group.move();
    }
    return error_code;
}
