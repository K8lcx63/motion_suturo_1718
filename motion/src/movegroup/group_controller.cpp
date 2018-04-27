#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <knowledge_msgs/GraspObject.h>
#include <knowledge_msgs/DropObject.h>
#include <knowledge_msgs/Gripper.h>
#include <motion_msgs/MovingCommandAction.h>
#include <group_controller.h>
#include <visualization_marker.h>

GroupController::GroupController(const ros::NodeHandle &nh) :
        nodeHandle (nh),
        gripperclient("gripper", true),
        planning_scene_controller (nh),
        visualizationMarker (nh)
        {
            robotStatePublisher = nodeHandle.advertise<moveit_msgs::DisplayRobotState> ("robot_state_at_ik_solution", 5);
            ikServiceClient = nodeHandle.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
            beliefstatePublisherGrasp = nodeHandle.advertise<knowledge_msgs::GraspObject>("/beliefstate/grasp_action", 10);
            beliefstatePublisherDrop = nodeHandle.advertise<knowledge_msgs::DropObject>("/beliefstate/drop_action", 10);
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

    geometry_msgs::PointStamped toVisualize;
    toVisualize.header.frame_id = goal_pose.header.frame_id;
    toVisualize.point = goal_pose.pose.position;
    visualizationMarker.publishVisualizationMarker(toVisualize, "motion-goal");

    geometry_msgs::PoseStamped goalPoseInPlanningFrame = point_transformer.transformPoseStamped(group.getPlanningFrame(), goal_pose);
    group.setPoseTarget(goalPoseInPlanningFrame);
    group.setGoalOrientationTolerance(0.1);
    group.setGoalPositionTolerance(0.05);
    group.setStartStateToCurrentState();
 
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
 
        secondGoalPose.pose.position.z += 0.2f;
 
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
                                          const geometry_msgs::PoseArray& objectGraspPoses, vector<string> poseDescription, double effort,
                                          std::string objectLabel) {

    //open gripper
    openGripper(motion_msgs::GripperGoal::LEFT);
    sleep(5);

    //remove old grasp-pose-meshes
    visualizationMarker.removeOldMeshes();

    //visualize possible grasp poses
    visualizationMarker.publishMeshes(objectGraspPoses, PATH_TO_GRIPPER_MESH);

    bool solutionFound = false;
    int i = 0;

    while (i < objectGraspPoses.poses.size() && !solutionFound) {
        moveit_msgs::GetPositionIK::Request ikRequest;
        ikRequest.ik_request.group_name = group.getName();


        ikRequest.ik_request.pose_stamped.header.frame_id = objectGraspPoses.header.frame_id;

        ikRequest.ik_request.pose_stamped.pose = objectGraspPoses.poses[i];

        ikRequest.ik_request.avoid_collisions = false;

        moveit_msgs::GetPositionIK::Response ikResponse;

        ikServiceClient.call(ikRequest, ikResponse);


        if (ikResponse.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

            /* Load the robot model */
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            /* Get a shared pointer to the model */
            robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
            /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */

            robot_state::RobotState robotState(kinematic_model);
            moveit::core::robotStateMsgToRobotState(ikResponse.solution, robotState);

            robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robotState));

            Eigen::Affine3d wristTransform;
            Eigen::Affine3d toolTransform;

            if(group.getName() == "right_arm"){
                wristTransform = kinematic_state->getFrameTransform("r_wrist_roll_link");
                toolTransform = kinematic_state->getFrameTransform("r_gripper_tool_frame");
            } else {
                wristTransform = kinematic_state->getFrameTransform("l_wrist_roll_link");
                toolTransform = kinematic_state->getFrameTransform("l_gripper_tool_frame");
            }

            tf::Pose wristTransformPose;
            tf::Pose toolTransformPose;
            tf::poseEigenToTF (wristTransform, wristTransformPose);
            tf::poseEigenToTF (toolTransform, toolTransformPose);

            tf::Vector3 directionToolToWristTransform;

            directionToolToWristTransform.setX(wristTransformPose.getOrigin().getX() - toolTransformPose.getOrigin().getX());
            directionToolToWristTransform.setY(wristTransformPose.getOrigin().getY() - toolTransformPose.getOrigin().getY());
            directionToolToWristTransform.setZ(wristTransformPose.getOrigin().getZ() - toolTransformPose.getOrigin().getZ());


            moveit_msgs::GetPositionIK::Request ikRequest;
            if (group.getName() == "right_arm") {
                ikRequest.ik_request.group_name = "right_arm";
            } else {
                ikRequest.ik_request.group_name = "left_arm";
            }


            geometry_msgs::PoseStamped goalPose;
            goalPose.header.frame_id = objectGraspPoses.header.frame_id;
            goalPose.header.stamp = ros::Time(0);

            goalPose.pose = objectGraspPoses.poses[i];

            geometry_msgs::PoseStamped goalInMapFrame = point_transformer.transformPoseStamped("map", goalPose);


            ikRequest.ik_request.pose_stamped.header.frame_id = goalInMapFrame.header.frame_id;
            ikRequest.ik_request.pose_stamped.pose.orientation = goalInMapFrame.pose.orientation;
            ikRequest.ik_request.pose_stamped.pose.position.x = goalInMapFrame.pose.position.x + directionToolToWristTransform.getX()*1.05f;
            ikRequest.ik_request.pose_stamped.pose.position.y = goalInMapFrame.pose.position.y + directionToolToWristTransform.getY()*1.05f;
            ikRequest.ik_request.pose_stamped.pose.position.z = goalInMapFrame.pose.position.z + directionToolToWristTransform.getZ()*1.05f;

            ikRequest.ik_request.avoid_collisions = false;

            moveit_msgs::GetPositionIK::Response ikResponse;

            ikServiceClient.call(ikRequest, ikResponse);


            if (ikResponse.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

                moveit::core::robotStateMsgToRobotState(ikResponse.solution, robotState);

                robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robotState));

                moveit_msgs::DisplayRobotState msg;
                robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

                robotStatePublisher.publish(msg);
                ros::spinOnce();

                sleep (2);

                group.setJointValueTarget(*kinematic_state);

                group.plan(execution_plan);

                if((group.plan(execution_plan)).val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                    //group.setGoalOrientationTolerance(0.8);
                    //group.setGoalPositionTolerance(0.3);
                    //if(group.move().val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                        //solutionFound = true;
                }
            }


        } else {
            ROS_ERROR("NO SOLUTION FOUND FOR THIS GRASP POSE!");
        }

        i++;
    }


    //pose selektieren etc..
    //ausführen
    /*
                if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                    if(group.getName() == "right_arm"){

                            closeGripper(motion_msgs::GripperGoal::RIGHT, effort);

                            if(!checkIfObjectGraspedSuccessfully(motion_msgs::GripperGoal::RIGHT)){
                                error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                return error_code;
                            } else {
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
                            closeGripper(motion_msgs::GripperGoal::LEFT, effort);

                            if(!checkIfObjectGraspedSuccessfully(motion_msgs::GripperGoal::LEFT)){
                                error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                return error_code;
                            } else {
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
 

                }
            }
        }
    }
    */
    moveit_msgs::MoveItErrorCodes result;
    result.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return result;
}

moveit_msgs::MoveItErrorCodes GroupController::dropObject(moveit::planning_interface::MoveGroup& group,
                                         const geometry_msgs::PoseStamped& object_drop_pose) {


    /*
    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        if(group.getName() == "right_arm"){
                openGripper(motion_msgs::GripperGoal::RIGHT);

                // publish message for beliefstate
                knowledge_msgs::DropObject msg;
                msg.gripper.gripper = knowledge_msgs::Gripper::RIGHT_GRIPPER;
                beliefstatePublisher.publish(msg);

                // detach object from gripper in planningscene
                planning_scene_controller.detachObject(objectLabel, "r_gripper_tool_frame");



        } else {
                openGripper(motion_msgs::GripperGoal::LEFT);

                // publish message for beliefstate
                knowledge_msgs::DropObject msg;
                msg.gripper.gripper = knowledge_msgs::Gripper::LEFT_GRIPPER;
                beliefstatePublisher.publish(msg);

                // detach object from gripper in planningscene
                planning_scene_controller.detachObject(objectLabel, "l_gripper_tool_frame");



        }


    }*/
    moveit_msgs::MoveItErrorCodes result;
    result.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return result;
}

bool GroupController::checkIfObjectGraspedSuccessfully(int gripperNum){

    // check for max 3.5 seconds, if gripper get's fully closed
    // if this is the case, the object was not successfully grasped
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(3.5);

    while(ros::Time::now() - start_time < timeout) {
        //get actual jointstate message
        sensor_msgs::JointState jointState = *(ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states",ros::Duration(1)));

        string gripperJoint = (gripperNum == motion_msgs::GripperGoal::LEFT ) ? "l_gripper_joint" : "r_gripper_joint";

        //find position of the right/left gripper joint to check if an object is inside
        int pos = find(jointState.name.begin(), jointState.name.end(), gripperJoint) - jointState.name.begin();

        if(pos < jointState.name.size()){
            string info = "GRIPPER JOINT VALUE : ";
            ROS_INFO(info.c_str());
            cout << jointState.position[pos] << endl;
            // if the joint value is not between these two values, the gripper didn't grasp an object
            if(!((jointState.position[pos] >= 0.005) && (jointState.position[pos] <= 0.08))){
                return false;
            }
        }
    }

    return true;
}

void GroupController::openGripper(int gripperNum){
    if (gripperclient.isServerConnected()) {
        motion_msgs::GripperGoal goal;
        goal.position = 0.09;
        goal.force = -1;
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
            goal.force = motion_msgs::MovingCommandGoal::FORCE_DEFAULT;
        } else {
            goal.force = effort;
        }
        goal.gripper = gripperNum;
        gripperclient.sendGoalAndWait(goal);

    } else {
        ROS_ERROR("GRIPPER SERVER NOT CONNECTED - ABORTED");
    }
}
