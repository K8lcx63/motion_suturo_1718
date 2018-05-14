#ifndef MOTION_ROSCONNECTOR_H
#define MOTION_ROSCONNECTOR_H

#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>

class ROSConnector {
public:
    moveit::planning_interface::MoveItErrorCode planMoveGroup(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &plan) {
        return group.plan(plan);
    }

    moveit::planning_interface::MoveItErrorCode moveMoveGroup(moveit::planning_interface::MoveGroup &group) {
        return group.move();
    }

    moveit::planning_interface::MoveItErrorCode executeMoveGroup(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::MoveGroup::Plan &plan) {
        return group.execute(plan);
    }

    template<typename T>
    static void publish(ros::Publisher &publisher, T &pubObj) {
        publisher.publish(pubObj);
    }
};

#endif //MOTION_ROSCONNECTOR_H
