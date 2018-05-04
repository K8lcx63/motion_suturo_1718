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
     * Publishes meshes with the given poses, the given colors and the given lifetimes.
     *
     * @param poses the pose of the meshes
     * @param frameId the frame the poses are given in.
     * @param ids the id the meshes shall get.
     * @param path the path of the mesh.
     * @param colors the colors the meshes shall have.
     * @param lifetimes the lifetimes the meshes shall have. If set to zero, lifetime is set to unlimited.
     */
    void publishMeshesWithColor(const std::vector<geometry_msgs::Pose> &poses, const std::string frameId, const std::vector<int> ids, std::string path,
                                                   std::vector<std_msgs::ColorRGBA> &colors, std::vector<ros::Duration> &lifetimes);

    /**
    * Removes all previously published meshes.
    */
    void removeOldMeshes();

    static visualization_msgs::Marker generatePoseMarker(const geometry_msgs::Pose &pose, const std::string &path, int id,
                                                  std::string frame);
};


#endif //SUTURO_MOTION_MAIN_VISUALIZATION_MARKER_H
