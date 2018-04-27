#ifndef SUTURO_MOTION_MAIN_VISUALIZATION_MARKER_H
#define SUTURO_MOTION_MAIN_VISUALIZATION_MARKER_H

#include <marker_publisher/marker_publisher.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <marker_publisher/marker_publisher.h>

/**
 * Class for publishing visualization markers.
 */
class VisualizationMarker {
private:
    MarkerPublisher markerPublisher;
    ros::NodeHandle nodeHandle;
    ros::Publisher visualizationMarkerPub;

public:
    /**
     * Constructor.
     *
     * @param nh NodeHandle.
     */
    VisualizationMarker(const ros::NodeHandle &nh);

    /**
     * Publishes a visualization marker.
     *
     * @param point of the visualization marker as pointStamped.
     * @param ns NameSpace of the VisualizationMarker.
     */
    void publishVisualizationMarker(const geometry_msgs::PointStamped& point, std::string ns);

    /**
     * Publishes a visualization marker.
     *
     * @param point of the visualization marker as pointStamped.
     * @param ns NameSpace of the VisualizationMarker.
     */
    void publishVisualizationMarkerWithColor(const geometry_msgs::PointStamped& point, std::string ns, const Color color);

    /**
     * Publishes a mesh for all the given poses.
     *
     * @param poses different poses of the mesh as PoseArray.
     * @param path the path of the mesh to spawn.
     */
    void publishMeshes(const geometry_msgs::PoseArray& pose, std::string path);

    /**
    * Removes all previously published meshes.
    */
    void removeOldMeshes ();
};


#endif //SUTURO_MOTION_MAIN_VISUALIZATION_MARKER_H
