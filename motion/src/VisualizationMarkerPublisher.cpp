#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include "VisualizationMarkerPublisher.h"

struct VisualizationMarkerPublisher::Private {
    static visualization_msgs::Marker createFrom(const geometry_msgs::PointStamped &point, const int markerType) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = point.header.frame_id;
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.point.x;
        marker.pose.position.y = point.point.y;
        marker.pose.position.z = point.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 0.7;
        if (markerType == TYPE_MOTION) {
            marker.ns = "motion";
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (markerType == TYPE_KNOWLEDGE) {
            marker.ns = "knowledge";
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        return marker;
    }
};

void VisualizationMarkerPublisher::publishVisualizationMarker(const geometry_msgs::PointStamped &point, const int markerType) {
    visualization_msgs::Marker marker = Private::createFrom(point, markerType);
    vis_pub.publish(marker);
}

void VisualizationMarkerPublisher::setVisPub(ros::Publisher pub) {
    vis_pub = pub;
}

VisualizationMarkerPublisher::VisualizationMarkerPublisher() {}
