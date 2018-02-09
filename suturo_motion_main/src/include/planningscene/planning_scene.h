#ifndef SUTURO_MOTION_MAIN_PLANNING_SCENE_H
#define SUTURO_MOTION_MAIN_PLANNING_SCENE_H


#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <ros/ros.h>
#include "../transform/point_transformer.h"

class PlanningSceneController {
private:
    PointTransformer transformer;
public:

    bool
    addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res, const std::string &planning_frame,
                               moveit::planning_interface::PlanningSceneInterface &planning_scene_interface);
};


#endif //SUTURO_MOTION_MAIN_PLANNING_SCENE_H
