//
// Created by menan on 12/17/17.
//

#ifndef MOTION_MARKERPUBLISHER_H
#define MOTION_MARKERPUBLISHER_H


#include <geometry_msgs/PointStamped.h>
#include <ros/publisher.h>

class MarkerPublisher {
public:
    /**
    * Publishes a visualization marker.
    *
    * @param point of the visualization marker as pointStamped.
    * @param color_schema ColorSchema, 0 = Red Point, 1 = Yellow Point.
    */
    static void publishVisualizationMarker(ros::Publisher &vis_pub, geometry_msgs::PointStamped point, int color_schema);
};


#endif //MOTION_MARKERPUBLISHER_H
