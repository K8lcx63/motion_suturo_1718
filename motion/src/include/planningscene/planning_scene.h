#ifndef SUTURO_MOTION_MAIN_PLANNING_SCENE_H
#define SUTURO_MOTION_MAIN_PLANNING_SCENE_H


#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <ros/ros.h>
#include "../transform/point_transformer.h"

/**
 * Class to control the planningscene.
 * Currently used to add the kitchen to the moveit planningscene.
 */
class PlanningSceneController {
private:
    PointTransformer transformer;
public:

    /**
     * Adds the Objects from the knowledge response to the moveit planning scene.
     *
     * @param res The knowledge response.
     * @param planning_frame The planningframe of the planningscene.
     * @param planning_scene_interface The planningscene interface.
     * @return true/false whether the objects could be added successful or not.
     */
    bool
    addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res, const std::string &planning_frame,
                               moveit::planning_interface::PlanningSceneInterface &planning_scene_interface);
};


#endif //SUTURO_MOTION_MAIN_PLANNING_SCENE_H
