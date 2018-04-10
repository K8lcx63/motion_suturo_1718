#include "../include/planningscene/planning_scene.h"

PlanningSceneController::PlanningSceneController(const ros::NodeHandle &nh) :
        node_handle(nh),
        robotModelLoader("robot_description"),
        planningScene(robotModelLoader.getModel())
{
    planningSceneDifferencePublisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
}

shape_msgs::Mesh PlanningSceneController::getMeshFromResource(const string meshPath){
    shapes::Mesh* mesh = shapes::createMeshFromResource(meshPath);
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(mesh,co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    return co_mesh;
}

bool PlanningSceneController::addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res,
                                                         const string& planning_frame) {
    int namesSize = res.names.size();
    int posesSize = res.poses.size();
    int boundingBoxesSize = res.bounding_boxes.size();

    if((namesSize != posesSize) || (posesSize != boundingBoxesSize) || (namesSize != boundingBoxesSize)){
        ROS_ERROR("Kitchen objects received from knowledge are inconsistent. - Aborted.");
        return false;
    }else{

        vector<moveit_msgs::CollisionObject> kitchenObjects;

        //add objects to collision matrix
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

            kitchenObjects.push_back(kitchenObject);
        }

        planning_scene_interface.addCollisionObjects(kitchenObjects);

        return true;
    }
}

bool PlanningSceneController::addPerceivedObjectToEnvironment(const knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr newPerceivedObject) {

    // for info on console
    string infoOutput;
    string errorOutput;

    infoOutput = "START TO ADD OBJECT " + newPerceivedObject->object_label + " TO THE PLANNING SCENE.";
    ROS_INFO (infoOutput.c_str());

    // check if data is valid
    if(!newPerceivedObject->object_label.empty() && !newPerceivedObject->pose.header.frame_id.empty() &&
            !newPerceivedObject->mesh_path.empty()) {

        infoOutput = "DATA SEEMS VALID, CONTINUING.";
        ROS_INFO (infoOutput.c_str());

        // create new CollisionObject-message to fill it with the data about the newly perceived object
        moveit_msgs::CollisionObject perceivedObject;

        // fill header
        perceivedObject.header.stamp = ros::Time::now();
        perceivedObject.header.frame_id = newPerceivedObject->pose.header.frame_id;
        perceivedObject.header.seq++;

        // fill in name
        perceivedObject.id = newPerceivedObject->object_label;

        // get mesh to add to planning scene
        shape_msgs::Mesh mesh = getMeshFromResource(newPerceivedObject->mesh_path);

        perceivedObject.meshes.push_back(mesh);

        // fill in pose of mesh
        geometry_msgs::Pose poseOfMesh;
        poseOfMesh.orientation = newPerceivedObject->pose.pose.orientation;
        poseOfMesh.position = newPerceivedObject->pose.pose.position;

        perceivedObject.mesh_poses.push_back(poseOfMesh);

        // define as operation to add a mesh to the environment
        perceivedObject.operation = perceivedObject.ADD;

        // publish PlanningScene-message to add mesh
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(perceivedObject);
        planning_scene.is_diff = true;
        planningSceneDifferencePublisher.publish(planning_scene);

        infoOutput = "SUCCESSFULLY ADDED OBJECT TO THE PLANNINGSCENE.";
        ROS_INFO(infoOutput.c_str());

        return true;
    }

    errorOutput = "COULD NOT ADD NEWLY PERCEIVED OBJECT " + newPerceivedObject->object_label + " TO PLANNINGSCENE BECAUSE"
                                                                                                       "OF INCORRECT DATA!";
    ROS_ERROR(errorOutput.c_str());
    return false;
}

bool PlanningSceneController::removeObjectFromEnvironment(const string objectName){

    if(objectName.empty()){
        ROS_ERROR ("CAN'T REMOVE OBJECT FROM PLANNINGSCENE-ENVIRONMENT, GIVEN NAME IS EMPTY!");
        return false;
    }

    // for info on console
    string infoOutput;

    infoOutput = "STARTING TO REMOVE OBJECT " + objectName + " FROM PLANNINGSCENE-ENVIRONMENT.";
    ROS_INFO (infoOutput.c_str());

    // create CollisionObject-message for defining the object to remove
    moveit_msgs::CollisionObject objectToRemove;
    objectToRemove.id = objectName;
    objectToRemove.header.frame_id = objectName;
    objectToRemove.operation = objectToRemove.REMOVE;

    // publish to apply removing the object
    moveit_msgs::PlanningScene planningScene;
    planningScene.world.collision_objects.push_back(objectToRemove);
    planningSceneDifferencePublisher.publish(planningScene);

    infoOutput = "SUCCESSFULLY REMOVED OBJECT FROM PLANNINGSCENE-ENVIRONMENT.";
    ROS_INFO (infoOutput.c_str());

    return true;
}

bool PlanningSceneController::attachObject(const string objectName, const string link){

    // for info on console
    string infoOutput;
    string errorOutput;

    if(!objectName.empty() && !link.empty()) {
        infoOutput = "DATA SEEMS VALID, START TO ATTACH OBJECT " + objectName + " TO LINK " + link + ".";
        ROS_INFO (infoOutput.c_str());

        // first remove the object from the environment
        removeObjectFromEnvironment(objectName);

        // then the object has to be attached to the robot

        // define the link the object has to be attached to
        moveit_msgs::AttachedCollisionObject attachedObject;
        attachedObject.link_name = link;

        attachedObject.object.header.frame_id = objectName;
        // define name and define that this is an add-operation
        attachedObject.object.id = objectName;
        attachedObject.object.operation = attachedObject.object.ADD;

        // set pose of object to zero position and identity orientation because the
        // header's frame id contains the frame of the object
        geometry_msgs::Pose poseOfMesh;
        poseOfMesh.orientation.w = 1.0;

        // fill mesh-path in message to send
        string meshPath = meshPathPrefix + objectName + "/" + objectName + ".dae";
        shape_msgs::Mesh mesh = getMeshFromResource(meshPath);

        attachedObject.object.meshes.push_back(mesh);
        attachedObject.object.mesh_poses.push_back(poseOfMesh);

        // publish message
        moveit_msgs::PlanningScene planningScene;
        planningScene.is_diff = true;
        planningScene.robot_state.is_diff = true;
        planningScene.robot_state.attached_collision_objects.push_back(attachedObject);
        planningSceneDifferencePublisher.publish(planningScene);

        infoOutput = "SUCCESSFULLY ATTACHED OBJECT " + objectName + " TO THE ROBOT AT LINK " + link + ".";
        ROS_INFO (infoOutput.c_str());

        return true;
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
        infoOutput = "DATA SEEMS VALID, START TO DETACH OBJECT " + objectName + " FROM LINK " + link + ".";
        ROS_INFO (infoOutput.c_str());

        //detach the object from the given link
        moveit_msgs::AttachedCollisionObject detachObject;
        detachObject.object.id = objectName;
        detachObject.link_name = link;
        detachObject.object.operation = detachObject.object.REMOVE;

        moveit_msgs::PlanningScene planningScene;
        planningScene.is_diff = true;
        planningScene.robot_state.is_diff = true;
        planningScene.robot_state.attached_collision_objects.push_back(detachObject);
        planningSceneDifferencePublisher.publish(planningScene);

        // re-introduce object to world

        knowledge_msgs::PerceivedObjectBoundingBox reintroduceObject;
        reintroduceObject.pose.header.frame_id = objectName;
        reintroduceObject.pose.header.stamp = ros::Time::now();

        geometry_msgs::Pose poseOfObject;
        poseOfObject.orientation.w = 1.0;

        reintroduceObject.pose.pose = poseOfObject;
        reintroduceObject.object_label = objectName;
        reintroduceObject.mesh_path = meshPathPrefix + objectName + "/" + objectName + ".dae";

        knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr reintroduceObjectPtr = knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr(
                &reintroduceObject);
        addPerceivedObjectToEnvironment(reintroduceObjectPtr);

        infoOutput = "SUCCESSFULLY DETACHED OBJECT " + objectName + " FROM LINK " + link + ".";
        ROS_INFO (infoOutput.c_str());

        return true;
    }

    errorOutput = "COULDN'T DETACH OBJECT " + objectName + " FROM LINK " + link + ", BECAUSE OF INVALID DATA.";
    ROS_ERROR (errorOutput.c_str());

    return false;
}

bool PlanningSceneController::setAllowCollision(const string objectName, const bool allowed){

    // for info on console
    string infoOutput;
    string errorOutput;

    if(!objectName.empty()){
        collision_detection::AllowedCollisionMatrix allowedCollisionMatrix = planningScene.getAllowedCollisionMatrix();

        allowedCollisionMatrix.setEntry(objectName, allowed);

        infoOutput = "SUCCESSFULLY ALLOWED COLLISION FOR OBJECT " + objectName + ".";
        ROS_INFO_STREAM(infoOutput.c_str());

        return true;
    }

    errorOutput = "NAME OF OBJECT IS EMPTY, UNABLE TO ALLOW COLLISION!";
    ROS_ERROR(errorOutput.c_str());

    return false;
}