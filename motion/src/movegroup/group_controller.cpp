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
        nodeHandle(nh),
        gripperclient("gripper", true),
        planning_scene_controller(nh),
        visualizationMarker(nh) {
    robotStatePublisher = nodeHandle.advertise<moveit_msgs::DisplayRobotState>("robot_state_at_ik_solution", 5);
    ikServiceClient = nodeHandle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    beliefstatePublisherGrasp = nodeHandle.advertise<knowledge_msgs::GraspObject>("/beliefstate/grasp_action", 10);
    beliefstatePublisherDrop = nodeHandle.advertise<knowledge_msgs::DropObject>("/beliefstate/drop_action", 10);
}

moveit_msgs::MoveItErrorCodes GroupController::moveArmsToDrivePose(moveit::planning_interface::MoveGroup &group) {
    group.setNamedTarget("arms_drive_pose");

    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        error_code = group.move();
    }

    return error_code;
}

moveit_msgs::MoveItErrorCodes
GroupController::moveGroupToCarryingObjectPose(moveit::planning_interface::MoveGroup &group) {

    std::string groupName = group.getName();

    if (groupName == "arms") {
        group.setNamedTarget("arms_carry_pose");
    } else if (groupName == "right_arm") {
        group.setNamedTarget("right_arm_carry_pose");
    } else if (groupName == "left_arm") {
        group.setNamedTarget("left_arm_carry_pose");
    }

    moveit_msgs::MoveItErrorCodes error_code = group.plan(execution_plan);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        error_code = group.move();
    }

    return error_code;
}

moveit_msgs::MoveItErrorCodes
GroupController::moveGroupToPose(moveit::planning_interface::MoveGroup &group,
                                 const geometry_msgs::PoseStamped &goal_pose) {

    geometry_msgs::PointStamped toVisualize;
    toVisualize.header.frame_id = goal_pose.header.frame_id;
    toVisualize.point = goal_pose.pose.position;
    visualizationMarker.publishVisualizationMarker(toVisualize, "motion-goal");

    geometry_msgs::PoseStamped goalPoseInPlanningFrame = point_transformer.transformPoseStamped(
            group.getPlanningFrame(), goal_pose);
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
    if (group.getName() == "right_arm") {
        firstGoalPose.pose.position.x -= GRIPPER_LENGTH_RIGHT;
    } else {
        firstGoalPose.pose.position.x -= GRIPPER_LENGTH_LEFT;
    }

    firstGoalPose.pose.position.x -= DISTANCE_BEFORE_POKING;

    // move to first goal point
    moveit_msgs::MoveItErrorCodes error_code = moveGroupToPose(group, firstGoalPose);

    /* If first movement was successful, calculate path to poke the object */
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

        geometry_msgs::PoseStamped secondGoalPose = point_transformer.transformPoseStamped("base_footprint",
                                                                                           object_middle);

        // put the second goal point some cm above the object's center
        // so that the object is most likely to be tilted
        // the robot will move his wrist to this goal point(the object to poke), and
        // the gripper is orientated to forward direction, so the robot will poke the
        // object with his gripper (will move 'thorugh' the object with his complete gripper)
        if (group.getName() == "left_arm_group") {
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
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

            // calculate direction for trajectory
            geometry_msgs::Vector3 directionVector;
            directionVector.x = secondGoalPose.pose.position.x - firstGoalPose.pose.position.x;
            directionVector.y = secondGoalPose.pose.position.y - firstGoalPose.pose.position.y;
            directionVector.z = secondGoalPose.pose.position.z - firstGoalPose.pose.position.z;

            // calculate the 10 sampled waypoints
            std::vector <geometry_msgs::Pose> waypoints;

            for (double i = 1; i <= 10; i++) {
                geometry_msgs::Pose waypoint;
                geometry_msgs::Vector3 step;

                step.x = directionVector.x * (i / 10.0);
                step.y = directionVector.y * (i / 10.0);
                step.z = directionVector.z * (i / 10.0);

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
            if (fraction != -1) {
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

moveit_msgs::MoveItErrorCodes GroupController::graspObject(moveit::planning_interface::MoveGroup &group,
                                                           const geometry_msgs::PoseArray &objectGraspPoses,
                                                           vector <string> poseDescription, double effort,
                                                           std::string objectLabel) {

    //open gripper
    openGripper(motion_msgs::GripperGoal::LEFT);
    sleep(2);

    //visualize possible grasp poses
    visualizationMarker.publishMeshes(objectGraspPoses, PATH_TO_GRIPPER_MESH);

    moveit_msgs::MoveItErrorCodes result;


    /* filtering and ranking of possible grasp poses */

    //for storing the result of the grasp pose filtering and ranking.
    //The list contains the indices of the grasp poses in 'objectGraspPoses'. This form of list is needed for
    //managing the spawned meshes of the different grasp poses in rviz, because the different meshes get the name
    //of their indice they have in the list 'objectGraspPoses' to differentiate between them.

    //After filtering the grasp poses, this list only contains the indices of the grasp poses, for which an ik solution
    //was found.
    //After ranking the left over grasp poses, this list is ordered the way the grasp poses were ranked. The first element
    //is the best ranked pose.
    vector<int> previousIndicesOfGraspPoses;

    //In this list, the ik solutions found for the different grasp poses are stored, in the same order as the list 'previousIndicesOfGraspPoses'
    vector <moveit_msgs::GetPositionIK::Response> ikSolutions;



    /* FILTERING GRASP POSES */

    //remove the grasp poses no ik solution is found for
    for (int i = 0; i < objectGraspPoses.poses.size(); i++) {
        //calculate goal for wrist frame from given goal for tool frame
        geometry_msgs::PoseStamped goalPose;
        goalPose.header.frame_id = objectGraspPoses.header.frame_id;
        goalPose.header.stamp = ros::Time(0);
        goalPose.pose = objectGraspPoses.poses[i];

        geometry_msgs::PoseStamped goalForWrist = point_transformer.transformPoseFromEndEffectorToWristFrame(goalPose,
                                                                                                             group);


        //create ik request
        moveit_msgs::GetPositionIK::Request ikRequest;
        moveit_msgs::GetPositionIK::Response ikResponse;

        ikRequest.ik_request.group_name = group.getName();
        ikRequest.ik_request.pose_stamped = goalForWrist;
        ikRequest.ik_request.avoid_collisions = true;

        std_msgs::ColorRGBA color;
        ros::Duration lifetime;

        //call function for finding the ik solution
        if (getIkSolution(ikRequest, ikResponse)) {
            previousIndicesOfGraspPoses.push_back(i);
            ikSolutions.push_back(ikResponse);

            //color grasp pose an ik solution was found for green
            color.a = 1.0;
            color.b = 0;
            color.r = 0;
            color.g = 1.0;

            lifetime = ros::Duration(0);
        } else {

            //color grasp pose no ik solution was found for red
            color.a = 1.0;
            color.b = 0;
            color.r = 1.0;
            color.g = 0;

            lifetime = ros::Duration(3);
        }

        visualizationMarker.publishMeshWithColor(objectGraspPoses.poses[i], objectGraspPoses.header.frame_id, i,
                                                 PATH_TO_GRIPPER_MESH, color, lifetime);
    }



    /* RANKING GRASP POSES */

    //the grasp poses an ik solution was found for now get ranked by
    // -> distance of the ik solution robot state to the next collision -> factor 0.65
    // -> distance of current robot state to robot state in ik solution -> factor 0.35
    motion_msgs::GripperGoal::_gripper_type gripper = (group.getName() == "right_arm") ? motion_msgs::GripperGoal::RIGHT : motion_msgs::GripperGoal::LEFT;
    robot_state::RobotStatePtr currentState = group.getCurrentState();

    rankGraspPoses(previousIndicesOfGraspPoses, ikSolutions, currentState, objectLabel, gripper);


    //color the meshes corresponding to their ranked order, starting with green and getting closer to red
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.g = 1.0;
    color.r = 0;
    color.b = 0;

    ros::Duration lifetime;

    for (int j = 0; j < previousIndicesOfGraspPoses.size(); j++) {
        color.r += 0.1 * j;
        color.g -= 0.1 * j;

        lifetime = ros::Duration(5);

        if(j == 0){
            lifetime = ros::Duration(0);
        }

        visualizationMarker.publishMeshWithColor(objectGraspPoses.poses[previousIndicesOfGraspPoses[j]],
                                                 objectGraspPoses.header.frame_id,
                                                 previousIndicesOfGraspPoses[j], PATH_TO_GRIPPER_MESH, color, lifetime);
    }



    /* EXECUTE BEST SOLUTION GRASP POSE*/

    //get all required information for executing
    int indexOfTakenSolution = previousIndicesOfGraspPoses[0];
    moveit_msgs::GetPositionIK::Response ikSolutionTaken = ikSolutions[0];

    geometry_msgs::PoseStamped goalPose;
    goalPose.header.frame_id = objectGraspPoses.header.frame_id;
    goalPose.header.stamp = ros::Time(0);
    goalPose.pose = objectGraspPoses.poses[indexOfTakenSolution];

    geometry_msgs::PoseStamped goalForWrist = point_transformer.transformPoseFromEndEffectorToWristFrame(goalPose, group);

    //visualize goal state of robot in rviz
    moveit::core::RobotStatePtr robotStateInIkSolution = visualizeIkSolution(ikSolutionTaken);

    //plan and execute the calculated ik solution
    group.setJointValueTarget(*robotStateInIkSolution);

    group.setGoalOrientationTolerance(0.05);
    group.setGoalPositionTolerance(0.03);

    result.val = group.plan(execution_plan);

    if (result.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

        result = group.move();

        if (result.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {

            //remove mesh for visualizing goal pose
            visualizationMarker.removeOldMeshes();

            //set some values depending on the used arm
            motion_msgs::GripperGoal gripperGoal;
            knowledge_msgs::Gripper graspGripper;
            string attachLink;

            if (group.getName() == "right_arm") {
                gripperGoal.gripper = motion_msgs::GripperGoal::RIGHT;
                graspGripper.gripper = knowledge_msgs::Gripper::RIGHT_GRIPPER;
                attachLink = "r_gripper_tool_frame";
            } else {
                gripperGoal.gripper = motion_msgs::GripperGoal::LEFT;
                graspGripper.gripper = knowledge_msgs::Gripper::LEFT_GRIPPER;
                attachLink = "l_gripper_tool_frame";
            }

            //grasp object
            closeGripper(gripperGoal.gripper, effort);

            //check if object was successfully grasped by checking the joint states of the gripper
            if (!checkIfObjectGraspedSuccessfully(gripperGoal.gripper)) {
                result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                return result;
            } else {
                //if successfully grasped, continue

                // publish message for beliefstate
                knowledge_msgs::GraspObject msg;
                msg.gripper.gripper = graspGripper.gripper;
                msg.object_label = objectLabel;
                msg.grasp_pose = goalPose;
                beliefstatePublisherGrasp.publish(msg);

                // attach object to gripper in planningscene
                planning_scene_controller.attachObject(objectLabel, attachLink);

                //temporarily allow collision for grasped object with actually colliding object (e.g. table)
                //until moved object away from table
                if (allowCollisionWithCollidingObjects(objectLabel, gripperGoal.gripper)) {

                    //move up for some cm's to get out of the collision of the grasped object
                    geometry_msgs::PoseStamped liftGoal = goalForWrist;
                    liftGoal.pose.position.z += LIFTING_AFTER_GRASPING;

                    result = moveGroupToPose(group, liftGoal);

                    if (!result.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                        result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                        return result;
                    }

                    //now the grasped object is not colliding with table anymore.
                    //in future, only allow the object to collide with gripper holding it
                    allowCollisionForGrasping(objectLabel, gripperGoal.gripper);

                } else {
                    ROS_ERROR("COULD NOT TEMPORARILY ALLOW COLLISION WITH COLLIDING OBJECTS, ABORTING.");
                    result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                    return result;
                }

            }
        }
    }

    //remove left over grasp pose meshes
    visualizationMarker.removeOldMeshes();

    return result;
}

moveit_msgs::MoveItErrorCodes GroupController::dropObject(moveit::planning_interface::MoveGroup &group,
                                                          const geometry_msgs::PoseStamped &object_drop_pose) {


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

bool GroupController::getIkSolution(const moveit_msgs::GetPositionIK::Request &ikRequest,
                                    moveit_msgs::GetPositionIK::Response &ikResponse) {

    //try max. MAX_ATTEMPTS_TO_GET_IK_SOLUTION times to get an ik solution
    for(int i = 0; i < MAX_ATTEMPTS_TO_GET_IK_SOLUTION; i++){
        if(ikServiceClient.call(ikRequest, ikResponse)){
            if(ikResponse.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                return true;
        }
    }

    return false;
}

moveit::core::RobotStatePtr GroupController::visualizeIkSolution(const moveit_msgs::GetPositionIK::Response &solution) {
    //get robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    //create robot state for calculated ik solution
    moveit::core::RobotState robotState(kinematic_model);
    robotStateMsgToRobotState(solution.solution, robotState);
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(robotState));

    //create and publish message with this robot state to visualize it in rviz
    moveit_msgs::DisplayRobotState msg;
    robotStateToRobotStateMsg(*kinematic_state, msg.state);

    robotStatePublisher.publish(msg);
    ros::spinOnce();
    return kinematic_state;
}

void GroupController::rankGraspPoses(vector<int> &indices, vector <moveit_msgs::GetPositionIK::Response> &solutions,
                                     const robot_state::RobotStatePtr &currentState, const string objectLabel, const int gripper) {

    if(indices.size() <= 1)
        return;

    //for storing the distances between the current state and the solution states
    vector<double> stateDistances;
    //for storing the distances of the solution states to the nearest collision (ignoring self collision)
    vector<double> distancesToCollision;

    //get robot model and initial state
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotState robotState(kinematic_model);

    //calculate distances between states and the distance to the closest collision

    //for calculating the distance to collision, the collision with the gripper to grasp the object with gets
    //allowed for a short time, because otherwise the distance to collision would be nearly the same for every
    //grasp pose (distance of grasped object to gripper it get's grasped with)
    allowCollisionForGrasping(objectLabel, gripper);

    for(int i = 0; i < solutions.size(); i++){
        stateDistances.push_back(getStateDistance(currentState, robotState, solutions[i].solution));
        distancesToCollision.push_back(planning_scene_controller.distanceToCollision(robotState, solutions[i].solution));
    }

    //the collision of the object with the gripper/robot in general get's forbidden again
    planning_scene_controller.addObjectToCollisionMatrix(objectLabel, false);

    //calculate the overall values
    //the state distance gets weighted with 0.35 and the distance to collision with 0.65
    vector<double> overall;

    for(int i = 0; i < solutions.size(); i++){
        //the overall value is calculated as follows:
        //  overallValue = stateDistance*0.35 - distanceToCollision*0.65
        //the distanceToCollision value is substracted and not added, because a higher distance to a collision
        //is better than a lower distance to a collision and in the end, the grasp pose with the lowest overall value
        //is ranked as the best grasp pose
        double overallValue = stateDistances[i]*0.35 - distancesToCollision[i]*0.65;
        overall.push_back(overallValue);
    }

    //rank grasp poses by evaluating the overall values
    //a small value is ranked better than a high value

    //bubble sort the three lists 'overall', 'indices' and 'solutions'
    int i, j,tmpOverall, tmpIndices;
    moveit_msgs::GetPositionIK::Response tmpSolutions;


    for (i = 0; i < overall.size() ; i++) {
        for (j = i; j < overall.size() ; j++) {

            if (overall[i] > overall[j]) {

                tmpOverall = overall[i];
                tmpIndices = indices[i];
                tmpSolutions = solutions[i];

                overall[i] = overall[j];
                indices[i] = indices[j];
                solutions[i] = solutions[j];

                overall[j] = tmpOverall;
                indices[j] = tmpIndices;
                solutions[j] = tmpSolutions;
            }
        }
    }
}

double GroupController::getStateDistance(const robot_state::RobotStatePtr &currentState, robot_state::RobotState &robotInitialState,
                                         const moveit_msgs::RobotState &solutionState){

    robotStateMsgToRobotState(solutionState, robotInitialState);
    const moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(robotInitialState));

    double distance;

    distance = currentState->distance(*kinematic_state);

    return distance;
}

bool GroupController::allowCollisionWithCollidingObjects(const string objectLabel, int gripper) {
    collision_detection::CollisionResult collision = planning_scene_controller.checkForCollision();

    vector <string> toAllowCollisionWith;

    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision.contacts.begin(); it != collision.contacts.end(); ++it) {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());

        //collect object names, grasped object is colliding with
        if (it->first.first.compare(objectLabel) == 0) {
            toAllowCollisionWith.push_back(it->first.second);
        } else if (it->first.second.compare(objectLabel) == 0) {
            toAllowCollisionWith.push_back(it->first.first);
        }
    }

    //also add all links of gripper

    vector <string> gripperLinks = getGripperLinks(gripper);

    vector<string>::iterator linkIt;
    for (int i = 0; i < gripperLinks.size(); i++) {
        linkIt = find(toAllowCollisionWith.begin(), toAllowCollisionWith.end(), gripperLinks[i]);

        //if not already in list, add it
        if (linkIt == toAllowCollisionWith.end())
            toAllowCollisionWith.push_back(gripperLinks[i]);
    }

    //apply allowing collision
    return planning_scene_controller.allowCollisionForSetOfObjects(objectLabel, toAllowCollisionWith);
}

bool GroupController::allowCollisionForGrasping(const string objectName, int gripper) {
    //allow collision with all links of gripper holding the object at the moment

    vector <string> gripperLinks = getGripperLinks(gripper);

    //apply allowing collision
    return planning_scene_controller.allowCollisionForSetOfObjects(objectName, gripperLinks);
}

bool GroupController::checkIfObjectGraspedSuccessfully(int gripperNum) {

    // check for max 3.5 seconds, if gripper get's fully closed
    // if this is the case, the object was not successfully grasped
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(3.5);

    while (ros::Time::now() - start_time < timeout) {
        //get actual jointstate message
        sensor_msgs::JointState jointState = *(ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states",
                                                                                                   ros::Duration(1)));

        string gripperJoint = (gripperNum == motion_msgs::GripperGoal::LEFT) ? "l_gripper_joint" : "r_gripper_joint";

        //find position of the right/left gripper joint to check if an object is inside
        int pos = find(jointState.name.begin(), jointState.name.end(), gripperJoint) - jointState.name.begin();

        if (pos < jointState.name.size()) {
            string info = "GRIPPER JOINT VALUE : ";
            ROS_INFO(info.c_str());
            cout << jointState.position[pos] << endl;
            // if the joint value is not between these two values, the gripper didn't grasp an object
            if (!((jointState.position[pos] >= 0.005) && (jointState.position[pos] <= 0.08))) {
                return false;
            }
        }
    }

    return true;
}

void GroupController::openGripper(int gripperNum) {
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

void GroupController::closeGripper(int gripperNum, double &effort) {
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

vector <string> GroupController::getGripperLinks(int gripper) {
    vector <string> result;

    if (gripper == motion_msgs::GripperGoal::RIGHT) {
        result.push_back("r_gripper_l_finger_link");
        result.push_back("r_gripper_l_finger_tip_link");
        result.push_back("r_gripper_motor_accelerometer_link");
        result.push_back("r_gripper_palm_link");
        result.push_back("r_gripper_r_finger_link");
        result.push_back("r_gripper_r_finger_tip_link");
    } else {
        result.push_back("l_gripper_l_finger_link");
        result.push_back("l_gripper_l_finger_tip_link");
        result.push_back("l_gripper_motor_accelerometer_link");
        result.push_back("l_gripper_palm_link");
        result.push_back("l_gripper_r_finger_link");
        result.push_back("l_gripper_r_finger_tip_link");
    }

    return result;
}
