#include "../include/visualization/visualization_marker.h"

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

        visualization_msgs::Marker marker;

        marker.header.frame_id = poses.header.frame_id;
        marker.header.stamp = ros::Time(0);
        marker.ns = "possible_grasp_poses";
        marker.id = i;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = poses.poses[i];

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.a = 0.8;
        marker.color.r = 1.0;
        marker.color.g = 0.8;
        marker.color.b = 0.8;

        marker.mesh_resource = path;

        visualizedPoses.markers.push_back(marker);
    }


    visualizationMarkerPub.publish( visualizedPoses );
}

void VisualizationMarker::removeOldMeshes (){
    visualization_msgs::MarkerArray removePoses;

    visualization_msgs::Marker marker;

    marker.ns = "possible_grasp_poses";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::DELETE;

    removePoses.markers.push_back(marker);

    visualizationMarkerPub.publish(removePoses);
}