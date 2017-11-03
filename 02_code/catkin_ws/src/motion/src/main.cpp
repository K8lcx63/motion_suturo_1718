#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion/MovingCommandAction.h>
#include <std_msgs/String.h>
#include <vector>


class Main {
private:
    ros::NodeHandle node_handle;
    moveit::planning_interface::MoveGroup group;
    geometry_msgs::Pose target_pose1;
    actionlib::SimpleActionServer<motion::MovingCommandAction> action_server;

public:
    Main(const ros::NodeHandle &nh) :
            node_handle(nh),
            group("right_arm"),
            action_server(node_handle, "moving", boost::bind(&Main::executeCommand, this, _1), false)
            {
            	action_server.start();
			}


	void executeCommand(const motion::MovingCommandGoalConstPtr &goal){
	    switch(goal->command) {
            case 1:
//                ROS_INFO("Moving to standard pose");
                //moveit::planning_interface::MoveGroup group("right_arm");
                //moveit::planning_interface::MoveGroup leftgroup("left_arm");

                target_pose1.orientation.w = 1.0;
                target_pose1.position.x = 0.28;
                target_pose1.position.y = -0.7;
                target_pose1.position.z = 1;
                group.setPoseTarget(target_pose1);

                /*geometry_msgs::Pose target_pose2;
                target_pose2.orientation.w = 1.0;
                target_pose2.position.x = 0.28;
                target_pose2.position.y = 0.7;
                target_pose2.position.z = 1;
                leftgroup.setPoseTarget(target_pose2);*/

                group.move();
                //leftgroup.move();

                break;
            case 2:
                ROS_INFO("Moving right arm");
                break;
            case 3:
                ROS_INFO("Moving left arm");
                break;
            default:
                ROS_INFO("COMMAND UNKNOWN");
                break;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_main");
    ros::NodeHandle node_handle;

    Main main(node_handle);
    ros::spin();
    /*ros::AsyncSpinner spinner(1);
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
    leftgroup.move();*/

    return 0;
}


