#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "../include/planningscene/planning_scene.h"

PlanningSceneController::PlanningSceneController(const ros::NodeHandle &nh) :
        node_handle(nh)
{
    planningSceneDifferencePublisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
}

bool PlanningSceneController::addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res,
                                                         const std::string& planning_frame) {
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
    std::string infoOutput;
    std::string errorOutput;

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

        // create mesh to add to planning scene
        shapes::Mesh* mesh = shapes::createMeshFromResource(newPerceivedObject->mesh_path);
        shape_msgs::Mesh co_mesh;
        shapes::ShapeMsg co_mesh_msg;
        shapes::constructMsgFromShape(mesh,co_mesh_msg);
        co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

        perceivedObject.meshes.push_back(co_mesh);

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

bool PlanningSceneController::removeObjectFromEnvironment(const std::string objectName){

    if(objectName.empty()){
        ROS_ERROR ("CAN'T REMOVE OBJECT FROM PLANNINGSCENE-ENVIRONMENT, GIVEN NAME IS EMPTY!");
        return false;
    }

    // for info on console
    std::string infoOutput;

    infoOutput = "STARTING TO REMOVE OBJECT " + objectName + " FROM PLANNINGSCENE-ENVIRONMENT.";
    ROS_INFO (infoOutput.c_str());

    // create CollisionObject-message for defining the object to remove
    moveit_msgs::CollisionObject objectToRemove;
    objectToRemove.id = objectName;
    objectToRemove.header.frame_id = "odom_combined";
    objectToRemove.operation = objectToRemove.REMOVE;

    // publish to apply removing the object
    moveit_msgs::PlanningScene planningScene;
    planningScene.world.collision_objects.push_back(objectToRemove);
    planningSceneDifferencePublisher.publish(planningScene);

    infoOutput = "SUCCESSFULLY REMOVED OBJECT FROM PLANNINGSCENE-ENVIRONMENT.";
    ROS_INFO (infoOutput.c_str());

    return true;
}

bool PlanningSceneController::allowCollision(const std::string objectName){
    return true;
}

bool PlanningSceneController::avoidCollision(const std::string objectName){
    return true;
}

bool PlanningSceneController::attachObject(const std::string objectName, const std::string link){

    // for info on console
    std::string infoOutput;
    std::string errorOutput;

    infoOutput = "START TO ATTACH OBJECT " + objectName + " TO LINK " + link + ".";
    ROS_INFO (infoOutput.c_str());

    //removeObjectFromEnvironment
    //attachToLink

    return true;
}

bool PlanningSceneController::detachObject(const std::string objectName, const std::string link){

    //detachFromLink
    //re-introduce object to world

    return true;
}