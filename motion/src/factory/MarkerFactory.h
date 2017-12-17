//
// Created by menan on 12/17/17.
//

#ifndef MOTION_MARKERFACTORY_H
#define MOTION_MARKERFACTORY_H


#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

class MarkerFactory {
public:
    static const int COLOR_SCHEMA_MOTION = 0;
    static const int COLOR_SCHEMA_KNOWLEDGE= 1;
    static visualization_msgs::Marker createVisMarker(const geometry_msgs::PointStamped &point, int color_schema);
};


#endif //MOTION_MARKERFACTORY_H
