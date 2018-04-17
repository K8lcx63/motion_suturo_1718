#include "../include/planningscene/planning_scene.h"

PlanningSceneController::PlanningSceneController(const ros::NodeHandle &nh) :
        node_handle(nh),
        sleep_t(0.005f),
        tf(new tf::TransformListener(ros::Duration(2.0)))
{
    attachObjectPublisher = node_handle.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    collisionObjectPublisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
}

bool PlanningSceneController::addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res,
                                                         const string& planning_frame) {

    int namesSize = res.names.size();
    int posesSize = res.poses.size();
    int boundingBoxesSize = res.bounding_boxes.size();

    if((namesSize != posesSize) || (posesSize != boundingBoxesSize) || (namesSize != boundingBoxesSize)){
        ROS_ERROR("Kitchen objects received from knowledge are inconsistent. - Aborted.");
        return false;
    } else{

        //add objects to planning scene
        for(int i = 0; i < namesSize; i++){
            string name(res.names[i]);
            geometry_msgs::Pose pose = res.poses[i];
            geometry_msgs::Vector3 boundingBox = res.bounding_boxes[i];

            if(res.frame_id != planning_frame){
               geometry_msgs::PoseStamped poseIn;
               poseIn.header.frame_id = res.frame_id;
               poseIn.pose = pose;
               geometry_msgs::PoseStamped poseOut = transformer.transformPoseStamped(planning_frame, poseIn);

               pose.orientation = poseOut.pose.orientation;
               pose.position = poseOut.pose.position;
            }

            moveit_msgs::CollisionObject kitchenObject;
            kitchenObject.header.frame_id = planning_frame;
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

            //publish message to add object
            collisionObjectPublisher.publish(kitchenObject);
            //sleep for some miliseconds to wait until object applied to planning scene
            sleep_t.sleep();
        }

        //check if the objects were successfully added
        for(int i = 0; i < namesSize; i++){
            if(!isInCollisionWorld(res.names[i])){
                ROS_ERROR("NOT ALL KITCHEN COLLISION OBJECTS WERE SUCCESSFULLY ADDED TO THE PLANNINGSCENE!");
                return false;
            }
        }

        string infoOutput = "\x1B[32m: ALL KITCHEN OBJECTS ARE NOW IN PLANNINGSCENE.";
        ROS_INFO (infoOutput.c_str());
        return true;

    }
}

bool PlanningSceneController::addPerceivedObjectToEnvironment(const knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr newPerceivedObject) {

    // for info on console
    string infoOutput;
    string errorOutput;

    infoOutput = "\x1B[32m: GOT NEWLY PERCEIVED OBJECT " + newPerceivedObject->object_label + " TO ADD IT TO THE PLANNING SCENE.";
    ROS_INFO (infoOutput.c_str());

    // check if data is valid
    if(!newPerceivedObject->object_label.empty() && !newPerceivedObject->pose.header.frame_id.empty() &&
            !newPerceivedObject->mesh_path.empty()) {

        infoOutput = "\x1B[32m: DATA SEEMS VALID, CONTINUING.";
        ROS_INFO (infoOutput.c_str());

        //call function to add the object to the planning scene
        return addObjectToEnvironment(newPerceivedObject->object_label, newPerceivedObject->mesh_path,
                                      newPerceivedObject->pose);

    }

    errorOutput = "COULD NOT ADD NEWLY PERCEIVED OBJECT " + newPerceivedObject->object_label + " TO PLANNINGSCENE BECAUSE"
                                                                                                       "OF INCORRECT DATA!";
    ROS_ERROR(errorOutput.c_str());
    return false;
}

bool PlanningSceneController::addObjectToEnvironment(const string objectName, const string meshPath, const
                                                    geometry_msgs::PoseStamped pose) {

    // for info on console
    string infoOutput;
    string errorOutput;

    infoOutput = "\x1B[32m: STARTING TO ADD OBJECT " + objectName + " TO PLANNINGSCENE-ENVIRONMENT.";
    ROS_INFO (infoOutput.c_str());

    // create new CollisionObject-message to fill it with the data about the newly perceived object
    moveit_msgs::CollisionObject perceivedObject;

    // fill header
    perceivedObject.header.stamp = ros::Time::now();
    perceivedObject.header.frame_id = pose.header.frame_id;
    perceivedObject.header.seq++;

    // fill in name
    perceivedObject.id = objectName;

    // get mesh to add to planning scene
    shape_msgs::Mesh mesh = getMeshFromResource(meshPath);

    perceivedObject.meshes.push_back(mesh);

    // fill in pose of mesh
    geometry_msgs::Pose poseOfMesh;
    poseOfMesh.orientation = pose.pose.orientation;
    poseOfMesh.position = pose.pose.position;

    perceivedObject.mesh_poses.push_back(poseOfMesh);

    // define as operation to add a mesh to the environment
    perceivedObject.operation = perceivedObject.ADD;

    //publish to apply adding new object
    collisionObjectPublisher.publish(perceivedObject);
    //sleep for some miliseconds for changes to be applied
    sleep_t.sleep();

    //check if the object was successfully added to the planning scene

    if(!isInCollisionWorld(objectName)){
        errorOutput = "OBJECT " + objectName + " COULD NOT BE ADDED TO THE PLANNING SCENE!";
        ROS_ERROR(errorOutput.c_str());
        return false;
    } else{
        infoOutput = "\x1B[32m: SUCCESSFULLY ADDED OBJECT " + objectName + " TO THE PLANNINGSCENE.";
        ROS_INFO(infoOutput.c_str());

        return true;
    }
}

bool PlanningSceneController::removeObjectFromEnvironment(const string objectName){

    if(objectName.empty()){
        ROS_ERROR ("CAN'T REMOVE OBJECT FROM PLANNINGSCENE-ENVIRONMENT, GIVEN NAME IS EMPTY!");
        return false;
    }

    // for info on console
    string infoOutput;
    string errorOutput;

    infoOutput = "\x1B[32m: STARTING TO REMOVE OBJECT " + objectName + " FROM PLANNINGSCENE-ENVIRONMENT.";
    ROS_INFO (infoOutput.c_str());

    // define object to be removed
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = objectName;
    remove_object.header.frame_id = "odom_combined";
    remove_object.operation = remove_object.REMOVE;
    // publish to apply removing of the object
    collisionObjectPublisher.publish(remove_object);
    // wait some miliseconds
    sleep_t.sleep();

    //check if the object was successfully removed from the planning scene

    if(!isInCollisionWorld(objectName)){
        infoOutput = "\x1B[32m: SUCCESSFULLY REMOVED OBJECT " + objectName + " FROM THE PLANNINGSCENE.";
        ROS_INFO(infoOutput.c_str());

        return true;
    } else{
        errorOutput = "OBJECT " + objectName + " COULD NOT BE REMOVED FROM THE PLANNING SCENE!";
        ROS_ERROR(errorOutput.c_str());
        return false;
    }
}

bool PlanningSceneController::attachObject(const string objectName, const string link){

    // for info on console
    string infoOutput;
    string errorOutput;

    if(!objectName.empty() && !link.empty()) {
        infoOutput = "\x1B[32m: DATA SEEMS VALID, START TO ATTACH OBJECT " + objectName + " TO LINK " + link + ".";
        ROS_INFO (infoOutput.c_str());

        // first remove the object from the environment
        if(!removeObjectFromEnvironment(objectName)){
            return false;
        }

        // then the object has to be attached to the robot

        // define the link the object has to be attached to
        moveit_msgs::AttachedCollisionObject attachedObject;
        attachedObject.link_name = link;

        attachedObject.object.header.frame_id = "base_footprint";
        // define name and define that this is an add-operation
        attachedObject.object.id = objectName;
        attachedObject.object.operation = attachedObject.object.ADD;

        // set pose of object to zero position and identity orientation because the
        // header's frame id contains the frame of the object
        geometry_msgs::Pose poseOfMesh = transformer.lookupTransformPose("base_footprint", objectName, ros::Time(0));

        // fill mesh-path in message to send
        string meshPath = meshPathPrefix + objectName + "/" + objectName + ".dae";
        shape_msgs::Mesh mesh = getMeshFromResource(meshPath);

        attachedObject.object.meshes.push_back(mesh);
        attachedObject.object.mesh_poses.push_back(poseOfMesh);

        // apply attaching object
        attachObjectPublisher.publish(attachedObject);
        // wait some miliseconds for changes to be applied
        sleep_t.sleep();

        //check if the object was successfully attached to the link

        if(isAttached(objectName, link)){
            infoOutput = "\x1B[32m: SUCCESSFULLY ATTACHED OBJECT " + objectName + " TO THE ROBOT AT LINK " + link + ".";
            ROS_INFO (infoOutput.c_str());

            return true;
        } else{
            errorOutput = "COULDN'T ATTACH OBJECT " + objectName + " TO THE ROBOT!";
            ROS_ERROR(errorOutput.c_str());

            return false;
        }

    }

    errorOutput = "CAN'T ATTACH OBJECT " + objectName + " TO THE ROBOT, BECAUSE DATA IS INVALID!";
    ROS_ERROR(errorOutput.c_str());

    return false;
}

bool PlanningSceneController::detachObject(const string objectName, const string link){

    // for info on console
    string infoOutput;
    string errorOutput;

    if(!objectName.empty() && !link.empty()) {
        infoOutput = "\x1B[32m: DATA SEEMS VALID, START TO DETACH OBJECT " + objectName + " FROM LINK " + link + ".";
        ROS_INFO (infoOutput.c_str());

        //detach the object from the given link
        moveit_msgs::AttachedCollisionObject detachObject;
        detachObject.object.id = objectName;
        detachObject.link_name = link;
        detachObject.object.operation = detachObject.object.REMOVE;

        // apply detaching object
        attachObjectPublisher.publish(detachObject);
        // wait some miliseconds for changes to be applied
        sleep_t.sleep();

        //check if the object was successfully detached from the link

        if(isAttached(objectName, link)){
            errorOutput = "COULDN'T DETACH OBJECT " + objectName + " FROM LINK " + link + "!";
            ROS_ERROR(errorOutput.c_str());

            return false;
        } else{
            infoOutput = "\x1B[32m: SUCCESSFULLY DETACHED OBJECT " + objectName + " FROM THE ROBOT LINK " + link + ".";
            ROS_INFO (infoOutput.c_str());
        }

        // call function for reintroducing the object to the planning scene
        string meshPath = meshPathPrefix + objectName + "/" + objectName + ".dae";
        geometry_msgs::Pose poseOfMesh = transformer.lookupTransformPose("base_footprint", objectName, ros::Time(0));
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_footprint";
        pose.header.stamp = ros::Time::now();
        pose.pose.orientation = poseOfMesh.orientation;
        pose.pose.position = poseOfMesh.position;
        if(!addObjectToEnvironment(objectName, meshPath, pose))
            return false;

        return true;
    }

    errorOutput = "COULDN'T DETACH OBJECT " + objectName + " FROM LINK " + link + ", BECAUSE OF INVALID DATA.";
    ROS_ERROR (errorOutput.c_str());

    return false;
}

bool PlanningSceneController::isInCollisionWorld(const string objectName) {

    //check if object was successfully added to the planning scene by getting the scene actually used
    //by the movegroup
    planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
    planningSceneMonitor->requestPlanningSceneState(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE);

    planning_scene_monitor::LockedPlanningSceneRW planningSceneRW(planningSceneMonitor);
    planningSceneRW.operator->()->getCurrentStateNonConst().update();

    planning_scene::PlanningScenePtr scene = planningSceneRW.operator->()->diff();
    collision_detection::WorldConstPtr world = scene->getCollisionWorld()->getWorld();

    //check if object is in collision world of scene
    return world->hasObject(objectName);
}

bool PlanningSceneController::isAttached(const string objectName, const string link) {

    //check if object was successfully attached to the link by getting the scene actually used
    //by the movegroup
    planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
    planningSceneMonitor->requestPlanningSceneState(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE);

    planning_scene_monitor::LockedPlanningSceneRW planningSceneRW(planningSceneMonitor);
    planningSceneRW.operator->()->getCurrentStateNonConst().update();
    planning_scene::PlanningScenePtr scene = planningSceneRW.operator->()->diff();

    moveit_msgs::PlanningScene sceneMsgs;
    scene->getPlanningSceneMsg(sceneMsgs);

    //check if the object is attached to the given link
    for(int i = 0; i < sceneMsgs.robot_state.attached_collision_objects.size(); i++){
        if(sceneMsgs.robot_state.attached_collision_objects[i].object.id == objectName &&
           sceneMsgs.robot_state.attached_collision_objects[i].link_name == link){
            return true;
        }

    }

    return false;
}

shape_msgs::Mesh PlanningSceneController::getMeshFromResource(const string meshPath){
    //create the mesh from a local file to be added into the planning scene
    shapes::Mesh* mesh = shapes::createMeshFromResource(meshPath);
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(mesh,co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    return co_mesh;
}