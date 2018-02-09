#ifndef SUTURO_MOTION_MAIN_VISUALIZATION_MARKER_H
#define SUTURO_MOTION_MAIN_VISUALIZATION_MARKER_H


#include <geometry_msgs/PointStamped.h>
#include <marker_publisher/marker_publisher.h>

class VisualizationMarker {
private:
    MarkerPublisher markerPublisher;
public:

    /**
     * Constructor.
     */
    VisualizationMarker();


    /**
     * Publishes a visualization marker.
     *
     * @param point of the visualization marker as pointStamped.
     * @param ns NameSpace of the VisualizationMarker.
     */
    void publishVisualizationMarker(geometry_msgs::PointStamped& point, std::string ns);
};


#endif //SUTURO_MOTION_MAIN_VISUALIZATION_MARKER_H
