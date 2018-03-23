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


bool PlanningSceneController::addPerceivedObjectToWorld(knowledge_msgs::PerceivedObjectBoundingBoxConstPtr newPerceivedObject) {
    return true;
}

bool allowCollision(const std::string object_label){
    return true;
}

bool avoidCollision(const std::string object_label){
    return true;
}

bool attachObject(const std::string object_label, move_group_interface::MoveGroup group){
    return true;
}

bool detachObject(const std::string object_label, move_group_interface::MoveGroup group){
    return true;
}