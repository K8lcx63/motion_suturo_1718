#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <iostream>

class Main {
private:
    ros::NodeHandle node_handle;

public:
    Main(const ros::NodeHandle &nh) :
            node_handle(nh) {

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement_example1");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Part of the robot to move
    moveit::planning_interface::MoveGroup group("right_arm");

    // specify that our target will be a random one
    group.setRandomTarget();

    // plan the motion and then move the group to the sampled target
    group.move();

    return 0;
}


