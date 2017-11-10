#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion/MovingCommandAction.h>
#include <motion/MovingCommandResult.h>
#include <std_msgs/String.h>
#include <string>
#include <geometry_msgs/Vector3.h>
#include <moveit_msgs/MoveItErrorCodes.h>


class Main {
private:
    ros::NodeHandle node_handle;
    moveit::planning_interface::MoveGroup right_arm_group;
    moveit::planning_interface::MoveGroup left_arm_group;
    moveit::planning_interface::MoveGroup both_arms;
    geometry_msgs::Pose target_pose1;
    actionlib::SimpleActionServer<motion::MovingCommandAction> action_server;
    moveit_msgs::MoveItErrorCodes error_code;
    motion::MovingCommandResult result;

public:
    Main(const ros::NodeHandle &nh) :
            node_handle(nh),
            right_arm_group("right_arm"),
            left_arm_group("left_arm"),
            both_arms("arms"),
            action_server(node_handle, "moving", boost::bind(&Main::executeCommand, this, _1), false)
            {
            	action_server.start();
			}


	void executeCommand(const motion::MovingCommandGoalConstPtr &goal){
        geometry_msgs::Vector3 vector(goal->vector);

        switch(goal->command) {
            case 1:
                ROS_INFO("Moving to initial pose.");
                both_arms.setNamedTarget("arms_initial");
                error_code = both_arms.move();
                break;
            case 2:
                ROS_INFO("Moving right arm to goal.");
                error_code = moveGroupToCoordinates(right_arm_group, vector);
                break;
            case 3:
                ROS_INFO("Moving left arm to goal.");
                error_code = moveGroupToCoordinates(left_arm_group, vector);
                break;
            default:
                ROS_ERROR("COMMAND UNKNOWN");
                result.successful = false;
                action_server.setAborted(result, "UNKNOWN COMMAND. ABORTED.");
                return;
        }

        if(error_code.val == error_code.SUCCESS){
            result.successful = true;
            action_server.setSucceeded(result);
            ROS_INFO("MOVE SUCCESSFUL");
        } else{
            result.successful = false;

            std::string error_string;
            std::ostringstream convert;

            convert << error_code.val;

            error_string = convert.str();

            action_server.setAborted(result, error_string);
            ROS_WARN("%s", error_string.c_str());
        }
    }

    moveit_msgs::MoveItErrorCodes moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, const geometry_msgs::Vector3 &vector) {
        target_pose1.orientation.w = 1.0;
        target_pose1.position.x = vector.x;
        target_pose1.position.y = vector.y;
        target_pose1.position.z = vector.z;
        group.setPoseTarget(target_pose1);
        return group.move();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_main");
    ros::NodeHandle node_handle;

    Main main(node_handle);
    ros::spin();

    return 0;
}


