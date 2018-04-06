#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <knowledge_msgs/GraspObject.h>
#include <knowledge_msgs/DropObject.h>
#include <knowledge_msgs/Gripper.h>
#include <motion_msgs/MovingCommandAction.h>
#include "../include/movegroup/group_controller.h"
#include "../include/visualization/visualization_marker.h"

GroupController::GroupController(const ros::NodeHandle &nh, const PlanningSceneController &planningSceneController) :
        gripperclient("gripper", true),
        planning_scene_controller (nh, planningSceneController)
        {
        }

moveit_msgs::MoveItErrorCodes GroupController::moveArmsToDrivePose(moveit::planning_interface::MoveGroup &group) {
    group.setNamedTarget("arms_drive_pose");
 
    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);
 
    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        error_code = group.move();
    }
 
    return error_code;
}

moveit_msgs::MoveItErrorCodes GroupController::moveGroupToCarryingObjectPose(moveit::planning_interface::MoveGroup &group) {

    std::string groupName = group.getName();

    if(groupName == "arms") {
        group.setNamedTarget("arms_carry_pose");
    } else if (groupName == "right_arm") {
        group.setNamedTarget("right_arm_carry_pose");
    } else if (groupName == "left_arm") {
        group.setNamedTarget("left_arm_carry_pose");
    }

    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);

    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        error_code = group.move();
    }

    return error_code;
}

moveit_msgs::MoveItErrorCodes
GroupController::moveGroupToPose(moveit::planning_interface::MoveGroup& group, const geometry_msgs::PoseStamped& goal_pose) {
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");
    ROS_INFO("PLANNING FROM REFACTORED CODE");

    geometry_msgs::PointStamped toVisualize;
    toVisualize.header.frame_id = goal_pose.header.frame_id;
    toVisualize.point = goal_pose.pose.position;
    visualizationMarker.publishVisualizationMarker(toVisualize, "motion-goal");

    geometry_msgs::PoseStamped goalPoseInPlanningFrame = point_transformer.transformPoseStamped(group.getPlanningFrame(), goal_pose);
    group.setPoseTarget(goalPoseInPlanningFrame);
    group.setGoalTolerance(0.005);
 
    moveit::planning_interface::MoveItErrorCode error_code = group.plan(execution_plan);
 
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        error_code = group.move();
    }
    return error_code;
}

moveit_msgs::MoveItErrorCodes GroupController::pokeObject(moveit::planning_interface::MoveGroup &group,
                                                                     const geometry_msgs::PoseStamped &object_middle) {
    /* First calculate pose to move group to front-direction of object to poke */
    // plan to move group in front of object
    geometry_msgs::PoseStamped firstGoalPose = point_transformer.transformPoseStamped("base_footprint", object_middle);
 
    // calculate position in front of object, so that the gripper tip link is DISTANCE_BEFORE_POKING away from object
    if(group.getName() == "right_arm"){
        firstGoalPose.pose.position.x -= GRIPPER_LENGTH_RIGHT;
    }else{
        firstGoalPose.pose.position.x -= GRIPPER_LENGTH_LEFT;
    }

    firstGoalPose.pose.position.x -= DISTANCE_BEFORE_POKING;
 
    // move to first goal point
    moveit_msgs::MoveItErrorCodes error_code = moveGroupToPose(group, firstGoalPose);

    /* If first movement was successful, calculate path to poke the object */
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

        geometry_msgs::PoseStamped secondGoalPose = point_transformer.transformPoseStamped("base_footprint", object_middle);

        // put the second goal point some cm above the object's center
        // so that the object is most likely to be tilted
        // the robot will move his wrist to this goal point(the object to poke), and
        // the gripper is orientated to forward direction, so the robot will poke the
        // object with his gripper (will move 'thorugh' the object with his complete gripper)
        if(group.getName() == "left_arm_group"){
            // move the goalpoint for the 'longer' arm closer to the robot (with an amount of
            // the difference between the two arm-length's) so that objects get poked
            // with right and left arm the same way (gripper through object)
            secondGoalPose.pose.position.x -= GRIPPER_LENGTH_LEFT - GRIPPER_LENGTH_RIGHT;
        }
 
        secondGoalPose.pose.position.z += 0.02f;
 
        // plan again to check if second goal pose can be reached by group
        group.setPoseTarget(secondGoalPose);
        group.setGoalTolerance(0.03);
        error_code = group.plan(execution_plan);

                // if point can be reached, calculate trajectory to point
        // so it is guaranteed that the robot moves his arm straight to the object following the trajectory
        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
 
            // calculate direction for trajectory
            geometry_msgs::Vector3 directionVector;
            directionVector.x = secondGoalPose.pose.position.x - firstGoalPose.pose.position.x;
            directionVector.y = secondGoalPose.pose.position.y - firstGoalPose.pose.position.y;
            directionVector.z = secondGoalPose.pose.position.z - firstGoalPose.pose.position.z;
 
            // calculate the 10 sampled waypoints
            std::vector<geometry_msgs::Pose> waypoints;
 
            for(double i = 1; i <= 10; i++){
                geometry_msgs::Pose waypoint;
                geometry_msgs::Vector3 step;
 
                step.x = directionVector.x * (i/10.0);
                step.y = directionVector.y * (i/10.0);
                step.z = directionVector.z * (i/10.0);
 
                waypoint.position.x = firstGoalPose.pose.position.x + step.x;
                waypoint.position.y = firstGoalPose.pose.position.y + step.y;
                waypoint.position.z = firstGoalPose.pose.position.z + step.z;

                waypoint.orientation = secondGoalPose.pose.orientation;
 
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

moveit_msgs::MoveItErrorCodes GroupController::graspObject(moveit::planning_interface::MoveGroup& group,
                                          const geometry_msgs::PoseStamped& object_grasp_pose, float effort, bool releaseObject,
                                            ros::Publisher beliefstatePublisher, std::string objectLabel) {

    /* Move arm to height which is up above all objects on table, so that collision is avoided */

    // create pose goal for moving group up above all objects
    float height;
    geometry_msgs::PoseStamped heightGoal;
    geometry_msgs::PointStamped actualPosition;
 
    if(group.getName() == "right_arm"){
        height = GroupController::TABLE_HEIGHT + MAXIMUM_OBJECT_HEIGHT + GRIPPER_LENGTH_RIGHT + DISTANCE_BEFORE_GRASPING;
        actualPosition = point_transformer.lookupTransform("base_footprint", "r_wrist_roll_link", ros::Time(0));
    } else{
        height = TABLE_HEIGHT + MAXIMUM_OBJECT_HEIGHT + GRIPPER_LENGTH_LEFT + DISTANCE_BEFORE_GRASPING;
        actualPosition = point_transformer.lookupTransform("base_footprint", "l_wrist_roll_link", ros::Time(0));
    }

    heightGoal.header.frame_id = actualPosition.header.frame_id;
    heightGoal.pose.position = actualPosition.point;
    heightGoal.pose.position.x = 0.3;

    if(group.getName() == "right_arm"){
        heightGoal.pose.position.y = -0.3;
    }else{
        heightGoal.pose.position.y = 0.3;
    }

    heightGoal.pose.position.z = height;
    heightGoal.pose.orientation.x = 0;
    heightGoal.pose.orientation.y = 0;
    heightGoal.pose.orientation.z = 0;
    heightGoal.pose.orientation.w = 1.0;

    moveit_msgs::MoveItErrorCodes error_code = moveGroupToPose(group, heightGoal);
 
 
    /* If successful, move arm above object, keeping the actual height */
 
    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        geometry_msgs::PoseStamped aboveObjectGoal = point_transformer.transformPoseStamped("base_footprint", object_grasp_pose);

        // change orientation
        aboveObjectGoal.pose.orientation.x = 0;
        aboveObjectGoal.pose.orientation.y = 0;
        aboveObjectGoal.pose.orientation.z = 0;
        aboveObjectGoal.pose.orientation.w = 1.0;
        // ignore height of goal and leave arm at it's actual height.
        aboveObjectGoal.pose.position.z = height;

        error_code = moveGroupToPose(group, aboveObjectGoal);
 
        /* If successful, set pose of arm to pose in 'object_grasp_pose' */
 
        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
            aboveObjectGoal.pose.orientation = object_grasp_pose.pose.orientation;

            error_code = moveGroupToPose(group, aboveObjectGoal);
 
            /* If successful, open/close left or right gripper, depending on which group is active and if releaseObject
             * is true or false. */
 
            if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                if(group.getName() == "right_arm"){
                    if(!releaseObject){
                        openGripper(motion_msgs::GripperGoal::RIGHT);
                    }

                } else{
                    if(!releaseObject){
                        openGripper(motion_msgs::GripperGoal::LEFT);
                    }
                }

                /* move to point from where gripper is able to grasp object */
 
                geometry_msgs::PoseStamped graspGoal = point_transformer.transformPoseStamped("base_footprint", object_grasp_pose);
                graspGoal.pose.orientation = aboveObjectGoal.pose.orientation;

                if(group.getName() == "right_arm"){
                    graspGoal.pose.position.z += GRIPPER_LENGTH_RIGHT;
                } else{
                    graspGoal.pose.position.z += GRIPPER_LENGTH_LEFT;
                }
 
                error_code = moveGroupToPose(group, graspGoal);
 
                /* If successful, close gripper to grasp object or open gripper to release object, depending on
                 * the value of releaseObject. */
 
                if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                    if(group.getName() == "right_arm"){
                        if(releaseObject){
                            openGripper(motion_msgs::GripperGoal::RIGHT);

                            // publish message for beliefstate
                            knowledge_msgs::DropObject msg;
                            msg.gripper.gripper = knowledge_msgs::Gripper::RIGHT_GRIPPER;
                            beliefstatePublisher.publish(msg);

                            // forbid collision with object in future
                            planning_scene_controller.setAllowCollision(objectLabel, false);

                            // detach object from gripper in planningscene
                            planning_scene_controller.detachObject(objectLabel, "r_gripper_tool_frame");
                        } else{
                            // allow collision for object
                            planning_scene_controller.setAllowCollision(objectLabel, true);

                            closeGripper(motion_msgs::GripperGoal::RIGHT, effort);

                            // publish message for beliefstate
                            knowledge_msgs::GraspObject msg;
                            msg.gripper.gripper = knowledge_msgs::Gripper::RIGHT_GRIPPER;
                            msg.object_label = objectLabel;
                            msg.grasp_pose = object_grasp_pose;
                            beliefstatePublisher.publish(msg);

                            // attach object to gripper in planningscene
                            planning_scene_controller.attachObject(objectLabel, "r_gripper_tool_frame");
                        }

                    } else {
                        if(releaseObject){
                            openGripper(motion_msgs::GripperGoal::LEFT);

                            // publish message for beliefstate
                            knowledge_msgs::DropObject msg;
                            msg.gripper.gripper = knowledge_msgs::Gripper::LEFT_GRIPPER;
                            beliefstatePublisher.publish(msg);

                            // forbid collision with object in future
                            planning_scene_controller.setAllowCollision(objectLabel, false);

                            // detach object from gripper in planningscene
                            planning_scene_controller.detachObject(objectLabel, "l_gripper_tool_frame");

                        } else{
                            // allow collision for object
                            planning_scene_controller.setAllowCollision(objectLabel, true);

                            closeGripper(motion_msgs::GripperGoal::LEFT, effort);

                            // publish message for beliefstate
                            knowledge_msgs::GraspObject msg;
                            msg.gripper.gripper = knowledge_msgs::Gripper::LEFT_GRIPPER;
                            msg.object_label = objectLabel;
                            msg.grasp_pose = object_grasp_pose;
                            beliefstatePublisher.publish(msg);

                            // attach object to gripper in planningscene
                            planning_scene_controller.attachObject(objectLabel, "l_gripper_tool_frame");
                        }

                    }
 
                    /* move arm again to height it had at the beginning to lift the object */

                     error_code = moveGroupToPose(group, aboveObjectGoal);
                }
            }
        }
    }

    return error_code;
}

void GroupController::openGripper(int gripperNum){
    if (gripperclient.isServerConnected()) {
        motion_msgs::GripperGoal goal;
        goal.position = 0.09;
        goal.effort = -1;
        goal.gripper = gripperNum;
        gripperclient.sendGoalAndWait(goal);
    } else {
        ROS_ERROR("GRIPPER SERVER NOT CONNECTED - ABORTED");
    }
}

void GroupController::closeGripper(int gripperNum, float& effort){
    if (gripperclient.isServerConnected()) {
        motion_msgs::GripperGoal goal;
        goal.position = 0.00;
        if (isnanf(effort)) {
            goal.effort = motion_msgs::MovingCommandGoal::FORCE_DEFAULT;
        } else {
            goal.effort = effort;
        }
        goal.gripper = gripperNum;
        gripperclient.sendGoalAndWait(goal);
    } else {
        ROS_ERROR("GRIPPER SERVER NOT CONNECTED - ABORTED");
    }
}
