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
    moveit::planning_interface::MoveGroup leftgroup("left_arm");

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.7;
    target_pose1.position.z = 1;
    group.setPoseTarget(target_pose1);

    geometry_msgs::Pose target_pose2;
    target_pose2.orientation.w = 1.0;
    target_pose2.position.x = 0.28;
    target_pose2.position.y = 0.7;
    target_pose2.position.z = 1;
    leftgroup.setPoseTarget(target_pose2);

    // specify that our target will be a random one
    //group.setRandomTarget();

    // plan the motion and then move the group to the sampled target
    group.move();
    leftgroup.move();

    return 0;
}


