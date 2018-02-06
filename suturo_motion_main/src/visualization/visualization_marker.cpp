#include "visualization_marker.h"
#include <marker_publisher/marker_publisher.h>

void VisualizationMarker::publishVisualizationMarker(geometry_msgs::PointStamped& point, std::string ns) {
    MarkerPublisher markerPublisher("motion", Color::RED);
    markerPublisher.publishVisualizationMarker(point);
}