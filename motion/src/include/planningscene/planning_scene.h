#ifndef SUTURO_MOTION_MAIN_PLANNING_SCENE_H
#define SUTURO_MOTION_MAIN_PLANNING_SCENE_H


#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <knowledge_msgs/PerceivedObjectBoundingBox.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <map>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "../transform/point_transformer.h"

/**
 * Class to control the planningscene.
 * Currently used to add the kitchen to the moveit planningscene.
 */

using namespace std;

class PlanningSceneController {
private:
    ros::NodeHandle node_handle;
    ros::Publisher planningSceneDifferencePublisher;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader robotModelLoader;
    planning_scene::PlanningScene planningScene;
    PointTransformer transformer;

    // to save the mesh-path's for the perceived objects
    map<string, string> object_meshes;
    // to save the object-frames on tf
    map<string, string> object_frames;

    /**
     * Returns a Mesh object for the mesh-file at the given path.
     * @param meshPath the path to the mesh-file.
     */
    shape_msgs::Mesh getMeshFromResource(const string meshPath);

public:

    /**
     * Copy-Constructor.
     * @param oldObject the PlanningSceneController object to copy.
     */
    PlanningSceneController(const PlanningSceneController &oldObject);

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
     * @return true/false whether the objects could be added or not.
     */
    bool
    addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res, const string &planning_frame);

    /**
     * Adds a mesh-collider into the planningscene for a newly perceived object.
     *
     * @param newPerceivedObject The data for the object to be added to the planningscene.
     * @return true/false whether the object could be added or not.
     */
    bool
    addPerceivedObjectToEnvironment(const knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr newPerceivedObject);

    /**
     * Removes the object with the given name from the environment of the plannningscene.
     *
     * @param objectName The name of the object to be removed from the planningscene.
     * @return true/false whether the object could be removed or not.
     */
    bool
    removeObjectFromEnvironment(const string objectName);

    /**
     * Allow or avoids collision with the object with the given label in the future, depending
     * on the value of the bool parameter.
     *
     * @param objectName the name of the object to allow/avoid collision for/with.
     * @param allowed if true, collision with object get's allowed in the future.
     *              If false, collision is avoided in the future.
     * @return true/false whether the collision was successfully allowed/avoided.
     */
    bool
    setAllowCollision(const string objectName, const bool allowed);

    /**
     * Attach object to robot after grasping it.
     *
     * @param objectName the name of the object to attach to the robot.
     * @param link the link the object shall be attached to.
     * @return true/false whether the object was successfully attached.
     */
    bool
    attachObject(const string objectName, const string link);

    /**
     * Detach object from robot after releasing it.
     *
     * @param objectName the name of the object to detach from the robot.
     * @param link the link the object shall be detached from.
     * @return true/false whether the object was successfully detached.
     */
    bool
    detachObject(const string objectName, const string link);
};


#endif //SUTURO_MOTION_MAIN_PLANNING_SCENE_H
