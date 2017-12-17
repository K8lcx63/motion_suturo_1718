//
// Created by menan on 12/17/17.

#ifndef MOTION_KITCHENCOLLISIONOBJECTSERVICE_H
#define MOTION_KITCHENCOLLISIONOBJECTSERVICE_H


#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>

class KitchenCollisionObjectService {
public:
    bool addKitchenCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,tf::TransformListener &tf, knowledge_msgs::GetFixedKitchenObjects::Response &res);
};


#endif //MOTION_KITCHENCOLLISIONOBJECTSERVICE_H
