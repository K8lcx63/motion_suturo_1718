#ifndef MOTION_VISUALIZATIONMARKERPUBLISHER_H
#define MOTION_VISUALIZATIONMARKERPUBLISHER_H


#include <geometry_msgs/PointStamped.h>

class VisualizationMarkerPublisher {
private:
    struct Private;
public:
    static const int TYPE_MOTION = 0;
    static const int TYPE_KNOWLEDGE = 1;

    /**
     * Publishes a visualization marker.
     *
     * @param point of the visualization marker as pointStamped.
     * @param color_schema ColorSchema, 0 = Red Point, 1 = Yellow Point.
     */
    static void publishVisualizationMarker(ros::Publisher &vis_pub, const geometry_msgs::PointStamped &point, const int &color_schema);
};


#endif //MOTION_VISUALIZATIONMARKERPUBLISHER_H
