#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include "../include/movegroup/group_controller.h"
#include "../include/visualization/visualization_marker.h"

moveit_msgs::MoveItErrorCodes moveArmsToInitial(moveit::planning_interface::MoveGroup &group) {
    group.setNamedTarget("arms_initial");

    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);

    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        error_code = group.move();
    }

    return error_code;
}

moveit_msgs::MoveItErrorCodes
moveGroupToCoordinates(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PointStamped& goal_point) {
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
    group.setGoalTolerance(0.03);

    moveit::planning_interface::MoveItErrorCode error_code = group.plan(execution_plan);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        error_code = group.move();
    }
    return error_code;
}

moveit_msgs::MoveItErrorCodes
moveGroupToPose(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PoseStamped& goal_pose) {
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");

    geometry_msgs::PoseStamped goalPoseInPlanningFrame = point_transformer.transformPoseStamped(group.getPlanningFrame(), goal_pose);
    group.setPoseTarget(goalPoseInPlanningFrame);
    group.setGoalTolerance(0.03);

    moveit::planning_interface::MoveItErrorCode error_code = group.plan(execution_plan);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        error_code = group.move();
    }
    return error_code;
}

moveit_msgs::MoveItErrorCodes pokeObject(moveit::planning_interface::MoveGroup &group,
                                                                     const geometry_msgs::PointStamped &object_middle) {


    /* First calculate pose to move group to front-direction of object to poke */


    // plan to move group in front of object
    geometry_msgs::PointStamped firstGoalPoint = point_transformer.transformPointStamped("base_footprint", object_middle);

    // calculate position in front of object, so that the gripper tip link is DISTANCE_BEFORE_POKING away from object
    if(group.getName() == "right_arm_group"){
        firstGoalPoint.point.x -= GRIPPER_LENGTH_RIGHT;
    }else{
        firstGoalPoint.point.x -= GRIPPER_LENGTH_LEFT;
    }

    firstGoalPoint.point.x -= DISTANCE_BEFORE_POKING;

    // move to first goal point
    moveit_msgs::MoveItErrorCodes error_code = moveGroupToCoordinates(group, firstGoalPoint);


    /* If first movement was successful, calculate path to poke the object */
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

        geometry_msgs::PointStamped secondGoalPoint = point_transformer.transformPointStamped("base_footprint", object_middle);

        // Move two-third of the gripper through the object so that the object is most likely to be tilted
        // Additionally put the second goal point some cm above the object's center
        if(group.getName() == "right_arm_group"){
            secondGoalPoint.point.x += GRIPPER_LENGTH_RIGHT * 2/3;
        }else{
            secondGoalPoint.point.x += GRIPPER_LENGTH_LEFT * 2/3;
        }

        secondGoalPoint.point.z += 0.02f;

        // plan again to check if second goal point can be reached by group
        geometry_msgs::PoseStamped secondGoalPose;
        secondGoalPose.header.frame_id = secondGoalPoint.header.frame_id;
        secondGoalPose.pose.position.x = secondGoalPoint.point.x;
        secondGoalPose.pose.position.y = secondGoalPoint.point.y;
        secondGoalPose.pose.position.z = secondGoalPoint.point.z;
        secondGoalPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);
        group.setPoseTarget(secondGoalPose);
        group.setGoalTolerance(0.03);
        error_code = group.plan(execution_plan);

        // if point can be reached, calculate trajectory to point
        // so it is guaranteed that the robot moves his arm straight to the object following the trajectory
        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){

            // calculate direction for trajectory
            geometry_msgs::Vector3 directionVector;
            directionVector.x = secondGoalPoint.point.x - firstGoalPoint.point.x;
            directionVector.y = secondGoalPoint.point.y - firstGoalPoint.point.y;
            directionVector.z = secondGoalPoint.point.z - firstGoalPoint.point.z;

            // calculate the 10 sampled waypoints
            std::vector<geometry_msgs::Pose> waypoints;

            for(double i = 1; i <= 10; i++){
                geometry_msgs::Pose waypoint;
                geometry_msgs::Vector3 step;

                step.x = directionVector.x * (i/10.0);
                step.y = directionVector.y * (i/10.0);
                step.z = directionVector.z * (i/10.0);

                waypoint.position.x = firstGoalPoint.point.x + step.x;
                waypoint.position.y = firstGoalPoint.point.y + step.y;
                waypoint.position.z = firstGoalPoint.point.z + step.z;

                waypoint.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0);

                waypoints.push_back(waypoint);

            }

            // set the calculated waypoints as goal-trajectory for group
            group.setPoseReferenceFrame("base_footprint");

            moveit_msgs::RobotTrajectory robotTrajectory;
            double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, robotTrajectory);

            // if computation was successful, execute the movement following the trajectory
            if(fraction != -1){
                robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
                rt.setRobotTrajectoryMsg(*group.getCurrentState(), robotTrajectory);

                rt.getRobotTrajectoryMsg(robotTrajectory);

                execution_plan.trajectory_ = robotTrajectory;
                error_code = group.execute(execution_plan);
            }
        }
    }

    return error_code;
}


moveit_msgs::MoveItErrorCodes graspObject(moveit::planning_interface::MoveGroup& group,
                                          const geometry_msgs::PointStamped& object_grasp_point) {

    /* Move arm to height which is up above all objects on table, so that collision is avoided */

    // create pose from 'object_grasp_point'. Possible in this case, because for now, we always grasp objects from above
    // and therefore the orientation is always the same
    geometry_msgs::PoseStamped object_grasp_pose;
    object_grasp_pose.header.stamp = ros::Time::now();
    object_grasp_pose.header.seq++;
    object_grasp_pose.header.frame_id = object_grasp_point.header.frame_id;
    object_grasp_pose.pose.position = object_grasp_point.point;
    object_grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, 0);

    // create pose goal for moving group up above all objects
    float height;
    geometry_msgs::PoseStamped heightGoal;
    geometry_msgs::PointStamped actualPosition;

    if(group.getName() == "right_arm_group"){
        height = TABLE_HEIGHT + MAXIMUM_OBJECT_HEIGHT + GRIPPER_LENGTH_RIGHT + DISTANCE_BEFORE_GRASPING;
        actualPosition = point_transformer.lookupTransform("r_wrist_roll_link", "base_footprint", ros::Time::now());
    } else{
        height = TABLE_HEIGHT + MAXIMUM_OBJECT_HEIGHT + GRIPPER_LENGTH_LEFT + DISTANCE_BEFORE_GRASPING;
        actualPosition = point_transformer.lookupTransform("l_wrist_roll_link", "base_footprint", ros::Time::now());
    }

    heightGoal.header.frame_id = actualPosition.header.frame_id;
    heightGoal.pose.position = actualPosition.point;
    heightGoal.pose.position.z = height;
    heightGoal.pose.orientation.x = 0;
    heightGoal.pose.orientation.y = 0;
    heightGoal.pose.orientation.z = 0;
    heightGoal.pose.orientation.w = 1;

    moveit_msgs::MoveItErrorCodes error_code = moveGroupToPose(group, heightGoal);


    /* If successful, move arm above object, keeping the actual height */

    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        geometry_msgs::PoseStamped aboveObjectGoal = point_transformer.transformPoseStamped("base_footprint", object_grasp_pose);

        // for now, keep orientation as it is now. Orientation will be changed later.
        aboveObjectGoal.pose.orientation.x = 0;
        aboveObjectGoal.pose.orientation.y = 0;
        aboveObjectGoal.pose.orientation.z = 0;
        aboveObjectGoal.pose.orientation.w = 1;
        // ignore height of goal and leave arm at it's actual height.
        aboveObjectGoal.pose.position.z = height;

        error_code = moveGroupToPose(group, aboveObjectGoal);

        /* If successful, set pose of arm to pose in 'object_grasp_pose' */

        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
            aboveObjectGoal.pose.orientation = object_grasp_pose.pose.orientation;

            error_code = moveGroupToPose(group, aboveObjectGoal);

            /* If successful, open left or right gripper, depending on which group is active */

            if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                if(group.getName() == "right_arm_group"){
                    openGripper("r_gripper_controller/gripper_action");
                } else{
                    openGripper("l_gripper_controller/gripper_action");
                }

                /* move to point from where gripper is able to grasp object */

                geometry_msgs::PoseStamped graspGoal = point_transformer.transformPoseStamped("base_footprint", object_grasp_pose);

                if(group.getName() == "right_arm_group"){
                    graspGoal.pose.position.z += GRIPPER_LENGTH_RIGHT;
                } else{
                    graspGoal.pose.position.z += GRIPPER_LENGTH_LEFT;
                }

                error_code = moveGroupToPose(group, graspGoal);

                /* If successful, close gripper to grasp object */

                if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                    if(group.getName() == "right_arm_group"){
                        closeGripper("r_gripper_controller/gripper_action");
                    } else{
                        closeGripper("l_gripper_controller/gripper_action");
                    }

                    /* move arm again to height it had at the beginning to lift the object */

                    error_code = moveGroupToPose(group, aboveObjectGoal);
                }
            }
        }
    }
}

void openGripper(std::string gripperName){
    typedef actionlib::SimpleActionClient <pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
    GripperClient *gripper_client_;
    gripper_client_ = new GripperClient(gripperName, true);

    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command
    open.command.position = 0.07;
    open.command.max_effort = -1;  // Do not limit effort (negative)
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The gripper opened!");
    } else {
        ROS_INFO("The gripper failed to open.");
        //ROS_INFO(gripper_client_->getState().toString().c_str());
        if(gripper_client_->isServerConnected()) {
            ROS_INFO("Client is connected");
        } else {
            ROS_INFO("Client NOT connected");
        }
    }
}

void closeGripper(std::string gripperName){
    typedef actionlib::SimpleActionClient <pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
    GripperClient *gripper_client_;
    gripper_client_ = new GripperClient(gripperName, true);

    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.01;
    squeeze.command.max_effort = -1;
    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult(ros::Duration(5.0));
    ROS_INFO("Waited 5 seconds for goal");
    if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The gripper closed!");
    } else {
        ROS_INFO("The gripper failed to close.");
        //ROS_INFO(gripper_client_->getState().toString().c_str());
        if(gripper_client_->isServerConnected()) {
            ROS_INFO("Client is connected");
        } else {
            ROS_INFO("Client NOT connected");
        }
    }
}