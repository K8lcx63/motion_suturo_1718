#ifndef SUTURO_MOTION_MAIN_PLANNING_SCENE_H
#define SUTURO_MOTION_MAIN_PLANNING_SCENE_H

#include <ros/ros.h>
#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <knowledge_msgs/PerceivedObjectBoundingBox.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>


#include "../transform/point_transformer.h"

/**
 * Class to control the planningscene.
 * Currently used to add the kitchen to the moveit planningscene.
 */

using namespace std;

class PlanningSceneController {
private:
    ros::NodeHandle node_handle;
    PointTransformer transformer;
    ros::Publisher attachObjectPublisher;
    ros::Publisher collisionObjectPublisher;
    ros::WallDuration sleep_t;

    boost::shared_ptr<tf::TransformListener> tf;


    const string meshPathPrefix = "package://knowledge_common/meshes/";

    /**
     * Returns a Mesh object for the mesh-file at the given path.
     * @param meshPath the path to the mesh-file.
     */
    shape_msgs::Mesh getMeshFromResource(const string meshPath);

    /**
     * Checks whether the object is in the collision world of the scene used by movegroup.
     * @param objectName the object to check for.
     */
    bool isInCollisionWorld(const string objectName);

    /**
     * Checks whether the object is attached/detached from the robot in the scene used by movegroup.
     * @param objectName the object to check for.
     * @param link the link the object should be attached to.
     */
    bool isAttached(const string objectName, const string link);

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
     * @return true/false whether the objects could be added or not.
     */
    bool
    addKitchenCollisionObjects(knowledge_msgs::GetFixedKitchenObjects::Response &res, const string &planning_frame);

    /**
     * Callback function.
     * Adds a mesh-collider into the planningscene for a newly perceived object.
     *
     * @param newPerceivedObject The data for the object to be added to the planningscene.
     * @return true/false whether the object could be added or not.
     */
    bool
    addPerceivedObjectToEnvironment(const knowledge_msgs::PerceivedObjectBoundingBox::ConstPtr newPerceivedObject);

    /**
     * Adds the object with the given name to the environment of the plannningscene.
     *
     * @param objectName The name of the object to be added to the planningscene.
     * @param meshPath the path of the mesh for the object.
     * @param pose the pose of the mesh to spawn in the planning scene.
     * @return true/false whether the object could be added or not.
     */
    bool
    addObjectToEnvironment(const string objectName, const string meshPath, const
    geometry_msgs::PoseStamped pose);

    /**
     * Removes the object with the given name from the environment of the plannningscene.
     *
     * @param objectName The name of the object to be removed from the planningscene.
     * @return true/false whether the object could be removed or not.
     */
    bool
    removeObjectFromEnvironment(const string objectName);

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
