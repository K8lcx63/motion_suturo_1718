#include "visualization_marker.h"
#include <marker_publisher/marker_publisher.h>

void VisualizationMarker::publishVisualizationMarker(geometry_msgs::PointStamped& point, std::string ns) {
    markerPublisher.publishVisualizationMarker(point);
}

VisualizationMarker::VisualizationMarker() :
markerPublisher("motion", Color::RED){}