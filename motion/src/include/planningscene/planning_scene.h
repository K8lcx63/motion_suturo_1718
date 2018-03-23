#ifndef SUTURO_MOTION_MAIN_PLANNING_SCENE_H
#define SUTURO_MOTION_MAIN_PLANNING_SCENE_H


#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <knowledge_msgs/PerceivedObjectBoundingBox.h>
#include <ros/ros.h>
#include "../transform/point_transformer.h"

/**
 * Class to control the planningscene.
 * Currently used to add the kitchen to the moveit planningscene.
 */
class PlanningSceneController {
private:
    ros::NodeHandle node_handle;
    ros::Publisher planningSceneDifferencePublisher;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    PointTransformer transformer;
public:

    /**
     * Constructor.
     * @param nh Node Handle.
     */
    PlanningSceneController(const ros::NodeHandle &nh);

    /**
     * Adds the Objects from the knowledge response to the moveit planning scene.
     *
     * @param res The knowledge response.
     * @param planning_frame The planningframe of the planningscene.
     * @param planning_scene_interface The planningscene interface.
     * @return true/false whether the objects could be added successful or not.
     */
    bool
    addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res, const std::string &planning_frame);

    /**
     * Adds a bounding box into the planningscene for the newly perceived object.
     *
     * @param newPerceivedObject The data for the object to be added to the planningscene.
     * @return true/false whether the object could be added successful or not.
     */
    bool
    addPerceivedObjectToWorld(const knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr newPerceivedObject);

    /**
     * Allow collision with the object with the given label in the future.
     *
     * @param object_label the name of the object to allow collision with.
     * @return true/false whether the collision was successfully allowed.
     */
    bool
    allowCollision(const std::string object_label);

    /**
     * Avoid collision with the object with the given label in the future.
     *
     * @param object_label the name of the object to avoid collision with.
     * @return true/false whether the collision was successfully avoided.
     */
    bool
    avoidCollision(const std::string object_label);

    /**
     * Attach object to robot after grasping it.
     *
     * @param object_label the name of the object to attach to the robot.
     * @param group the group (left arm or right arm group) to attach the object to.
     * @return true/false whether the object was successfully attached.
     */
    bool
    attachObject(const std::string object_label, move_group_interface::MoveGroup group);

    /**
     * Detach object from robot after releasing it.
     *
     * @param object_label the name of the object to detach from the robot.
     * @param group the group (left arm or right arm group) to detach the object from.
     * @return true/false whether the object was successfully detached.
     */
    bool
    detachObject(const std::string object_label, move_group_interface::MoveGroup group);
};


#endif //SUTURO_MOTION_MAIN_PLANNING_SCENE_H
