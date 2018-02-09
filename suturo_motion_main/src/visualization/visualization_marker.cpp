#include "visualization_marker.h"
#include <marker_publisher/marker_publisher.h>

VisualizationMarker::VisualizationMarker() :
        markerPublisher("motion", Color::RED){}

void VisualizationMarker::publishVisualizationMarker(const geometry_msgs::PointStamped& point, std::string ns) {
    markerPublisher.publishVisualizationMarker(point);
}

void VisualizationMarker::publishVisualizationMarkerWithColor(const geometry_msgs::PointStamped& point, std::string ns, const Color color) {
    markerPublisher.publishVisualizationMarkerWithColor(point, color);
}

