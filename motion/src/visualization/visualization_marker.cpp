#include <visualization_marker.h>


VisualizationMarker::VisualizationMarker(const ros::NodeHandle &nh) :
        nodeHandle(nh),
        markerPublisher("motion", Color::RED)
{
    visualizationMarkerPub = nodeHandle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
}

void VisualizationMarker::publishVisualizationMarker(const geometry_msgs::PointStamped& point, std::string ns) {
    markerPublisher.publishVisualizationMarker(point);
}

void VisualizationMarker::publishVisualizationMarkerWithColor(const geometry_msgs::PointStamped& point, std::string ns, const Color color) {
    markerPublisher.publishVisualizationMarkerWithColor(point, color);
}

void VisualizationMarker::publishMeshes(const geometry_msgs::PoseArray &poses, std::string path){

    visualization_msgs::MarkerArray visualizedPoses;

    for(int i = 0; i < poses.poses.size(); i++){

        visualization_msgs::Marker marker = generatePoseMarker(poses.poses[i], path, i, poses.header.frame_id);

        visualizedPoses.markers.push_back(marker);
    }


    visualizationMarkerPub.publish( visualizedPoses );
}

visualization_msgs::Marker
VisualizationMarker::generatePoseMarker(const geometry_msgs::Pose &pose, const std::string &path, int id, std::string frame) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time(0);
    marker.ns = "gripper_meshes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;

    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;

    marker.mesh_resource = path;
    return marker;
}

void VisualizationMarker::publishMeshesWithColor(const std::vector<geometry_msgs::Pose> &poses, const std::string frameId, const std::vector<int> ids, std::string path,
                                               std::vector<std_msgs::ColorRGBA> &colors, std::vector<ros::Duration> &lifetimes){

    visualization_msgs::MarkerArray newMeshes;

    for(int i = 0; i < poses.size(); i++){

        visualization_msgs::Marker marker;

        marker.header.frame_id = frameId;
        marker.header.stamp = ros::Time(0);
        marker.ns = "gripper_meshes";
        marker.id = ids[i];
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = poses[i];

        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 0.8;

        marker.color = colors[i];

        marker.mesh_resource = path;

        if(lifetimes[i].sec > 0)
            marker.lifetime = lifetimes[i];


        newMeshes.markers.push_back(marker);
    }


    visualizationMarkerPub.publish(newMeshes);

}

void VisualizationMarker::removeOldMeshes (){
    visualization_msgs::MarkerArray removePoses;
    visualization_msgs::Marker marker = generateDeleteMarker();

    removePoses.markers.push_back(marker);
    visualizationMarkerPub.publish(removePoses);
}

visualization_msgs::Marker VisualizationMarker::generateDeleteMarker() {
    visualization_msgs::Marker marker;
    marker.ns = "gripper_meshes";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
}