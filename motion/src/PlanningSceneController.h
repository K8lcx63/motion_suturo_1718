#ifndef MOTION_PLANNINGSCENECONTROLLER_H
#define MOTION_PLANNINGSCENECONTROLLER_H


#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <tf/transform_listener.h>

class PlanningSceneController {
private:
    tf::TransformListener listener;
public:
    /**
     * Adds the KitchenObjects from the response to the given {@link moveit::planning_interface::PlanningSceneInterface}.
     * @param res The Response from the message.
     * @param planning_scene_interface The PlanningSceneInterface to add objects
     * @return true/false whether it objects were added successful or not.
     */
    bool addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response& res,
                                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                    const std::string& planning_frame);
};


#endif //MOTION_PLANNINGSCENECONTROLLER_H
