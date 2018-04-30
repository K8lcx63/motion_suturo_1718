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

    moveit_msgs::MoveItErrorCodes result;

    while (i < objectGraspPoses.poses.size() && !solutionFound) {

        //calculate goal for wrist frame from given goal for tool frame
        geometry_msgs::PoseStamped goalPose;
        goalPose.header.frame_id = objectGraspPoses.header.frame_id;
        goalPose.header.stamp = ros::Time(0);
        goalPose.pose = objectGraspPoses.poses[i];

        geometry_msgs::PoseStamped goalForWrist = point_transformer.transformPoseFromEndEffectorToWristFrame(goalPose, group);


        //create ik request
        moveit_msgs::GetPositionIK::Request ikRequest;
        moveit_msgs::GetPositionIK::Response ikResponse;

        ikRequest.ik_request.group_name = group.getName();
        ikRequest.ik_request.pose_stamped = goalForWrist;
        ikRequest.ik_request.avoid_collisions = true;

        //send ik request
        bool success = ikServiceClient.call(ikRequest, ikResponse);

        if(success){
            if (ikResponse.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

                //get robot model
                robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
                robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

                //create robot state for calculated ik solution
                robot_state::RobotState robotState(kinematic_model);
                moveit::core::robotStateMsgToRobotState(ikResponse.solution, robotState);
                robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robotState));

                //create and publish message with this robot state to visualize it in rviz
                moveit_msgs::DisplayRobotState msg;
                robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

                robotStatePublisher.publish(msg);
                ros::spinOnce();
                sleep(2);

                //execute the calculated ik solution
                group.setJointValueTarget(*kinematic_state);

                group.plan(execution_plan);

                if((group.plan(execution_plan)).val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                    group.setGoalOrientationTolerance(0.1);
                    group.setGoalPositionTolerance(0.05);

                    if(group.move().val == moveit_msgs::MoveItErrorCodes::SUCCESS){

                        if(group.getName() == "right_arm"){

                            closeGripper(motion_msgs::GripperGoal::RIGHT, effort);

                            if(!checkIfObjectGraspedSuccessfully(motion_msgs::GripperGoal::RIGHT)){
                                result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                return result;
                            } else {
                                // publish message for beliefstate
                                knowledge_msgs::GraspObject msg;
                                msg.gripper.gripper = knowledge_msgs::Gripper::RIGHT_GRIPPER;
                                msg.object_label = objectLabel;
                                msg.grasp_pose = goalForWrist;
                                beliefstatePublisherGrasp.publish(msg);

                                // attach object to gripper in planningscene
                                planning_scene_controller.attachObject(objectLabel, "r_gripper_tool_frame");

                                //temporarily allow collision for grasped object with actually colliding object (e.g. table)
                                //until moved object away from table
                                if(allowCollisionWithCollidingObjects(objectLabel, knowledge_msgs::Gripper::RIGHT_GRIPPER)){
                                    //move up for some cm's to get out of the collision
                                    geometry_msgs::PoseStamped liftGoal = goalForWrist;
                                    liftGoal.pose.position.z += LIFTING_AFTER_GRASPING;
                                    if(!moveGroupToPose(group, liftGoal).val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                                        result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                        return result;
                                    }

                                    //now the grasped object is not colliding with table anymore.
                                    //in future, only allow the object to collide with gripper holding it
                                    allowCollisionForGrasping(objectLabel, knowledge_msgs::Gripper::RIGHT_GRIPPER);

                                    solutionFound = true;
                                } else{
                                    ROS_ERROR("COULD NOT TEMPORARILY ALLOW COLLISION WITH COLLIDING OBJECTS, ABORTING.");
                                    result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                    return result;
                                }

                            }



                        } else {

                            closeGripper(motion_msgs::GripperGoal::LEFT, effort);

                            if(!checkIfObjectGraspedSuccessfully(motion_msgs::GripperGoal::LEFT)){
                                result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                return result;
                            } else {
                                // publish message for beliefstate
                                knowledge_msgs::GraspObject msg;
                                msg.gripper.gripper = knowledge_msgs::Gripper::LEFT_GRIPPER;
                                msg.object_label = objectLabel;
                                msg.grasp_pose = goalForWrist;
                                beliefstatePublisherGrasp.publish(msg);

                                // attach object to gripper in planningscene
                                planning_scene_controller.attachObject(objectLabel, "l_gripper_tool_frame");

                                //temporarily allow collision for grasped object with actually colliding object (e.g. table)
                                //until moved object away from table
                                if(allowCollisionWithCollidingObjects(objectLabel, knowledge_msgs::Gripper::LEFT_GRIPPER)){
                                    //move up for some cm's to get out of the collision
                                    geometry_msgs::PoseStamped liftGoal = goalForWrist;
                                    liftGoal.pose.position.z += LIFTING_AFTER_GRASPING;
                                    if(!moveGroupToPose(group, liftGoal).val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                                        result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                        return result;
                                    }

                                    //now the grasped object is not colliding with table anymore.
                                    //in future, only allow the object to collide with gripper holding it
                                    allowCollisionForGrasping(objectLabel, knowledge_msgs::Gripper::LEFT_GRIPPER);

                                    solutionFound = true;
                                } else{
                                    ROS_ERROR("COULD NOT TEMPORARILY ALLOW COLLISION WITH COLLIDING OBJECTS, ABORTING.");
                                    result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                                    return result;
                                }

                            }


                        }
                    }
                }

            } else {
                ROS_ERROR_STREAM("NO SOLUTION FOUND FOR GRASP POSE " + to_string(i) + "!");
            }
        }

        i++;
    }


    if(solutionFound){
        result.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    } else {
        result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }

    return result;
}

moveit_msgs::MoveItErrorCodes GroupController::dropObject(moveit::planning_interface::MoveGroup& group,
                                         const geometry_msgs::PoseStamped& object_drop_pose) {


    /*
     * //kollision mit gripper nach dem abstellen wieder verbieten
     * planning_scene_controller.addObjectToCollisionMatrix(objectLabel, false);
     *
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

bool GroupController::allowCollisionWithCollidingObjects(const string objectLabel, int gripper){
    collision_detection::CollisionResult collision = planning_scene_controller.checkForCollision();

    vector<string> toAllowCollisionWith;

    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision.contacts.begin(); it != collision.contacts.end(); ++it)
    {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());

        //collect object names, grasped object is colliding with
        if(it->first.first.compare(objectLabel) == 0){
            toAllowCollisionWith.push_back(it->first.second);
        } else if(it->first.second.compare(objectLabel) == 0){
            toAllowCollisionWith.push_back(it->first.first);
        }
    }

    //also add all links of gripper

    vector<string> gripperLinks = getGripperLinks(gripper);

    vector<string>::iterator linkIt;
    for(int i = 0; i < gripperLinks.size(); i++){
        linkIt = find(toAllowCollisionWith.begin(), toAllowCollisionWith.end(), gripperLinks[i]);

        //if not already in list, add it
        if (linkIt == toAllowCollisionWith.end())
            toAllowCollisionWith.push_back(gripperLinks[i]);
    }

    //apply allowing collision
    return planning_scene_controller.allowCollisionForSetOfObjects(objectLabel, toAllowCollisionWith);
}

bool GroupController::allowCollisionForGrasping(const string objectName, int gripper){
    //allow collision with all links of gripper holding the object at the moment

    vector<string> gripperLinks = getGripperLinks(gripper);

    //apply allowing collision
    return planning_scene_controller.allowCollisionForSetOfObjects(objectName, gripperLinks);
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

void GroupController::closeGripper(int gripperNum, double& effort){
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

vector<string> GroupController::getGripperLinks(int gripper){
    vector<string> result;

    if(gripper == knowledge_msgs::Gripper::RIGHT_GRIPPER){
        result.push_back("r_gripper_l_finger_link");
        result.push_back("r_gripper_l_finger_tip_link");
        result.push_back("r_gripper_motor_accelerometer_link");
        result.push_back("r_gripper_palm_link");
        result.push_back("r_gripper_r_finger_link");
        result.push_back("r_gripper_r_finger_tip_link");
    }else{
        result.push_back("l_gripper_l_finger_link");
        result.push_back("l_gripper_l_finger_tip_link");
        result.push_back("l_gripper_motor_accelerometer_link");
        result.push_back("l_gripper_palm_link");
        result.push_back("l_gripper_r_finger_link");
        result.push_back("l_gripper_r_finger_tip_link");
    }

    return result;
}
