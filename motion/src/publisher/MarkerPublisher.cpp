//
// Created by menan on 12/17/17.
//

#include <visualization_msgs/Marker.h>
#include "MarkerPublisher.h"
#include "../factory/MarkerFactory.h"


void MarkerPublisher::publishVisualizationMarker(ros::Publisher &vis_pub, geometry_msgs::PointStamped point, int color_schema) {
    visualization_msgs::Marker marker = MarkerFactory::createVisMarker(point, color_schema);
    vis_pub.publish(marker);
}