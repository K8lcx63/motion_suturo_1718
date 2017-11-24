#include <ros/node_handle.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_msgs/MovingCommandAction.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>


class Main {
private:
    ros::NodeHandle node_handle;
    moveit::planning_interface::MoveGroup right_arm_group;
    moveit::planning_interface::MoveGroup left_arm_group;
    moveit::planning_interface::MoveGroup both_arms;
    geometry_msgs::Pose target_pose1;
    actionlib::SimpleActionServer<motion_msgs::MovingCommandAction> action_server;
    moveit_msgs::MoveItErrorCodes error_code;
    motion_msgs::MovingCommandResult result;
    tf::TransformListener listener;
    tf::StampedTransform transform;

public:
    Main(const ros::NodeHandle &nh) :
            node_handle(nh),
            right_arm_group("right_arm"),
            left_arm_group("left_arm"),
            both_arms("arms"),
            action_server(node_handle, "moving", boost::bind(&Main::executeCommand, this, _1), false) {
            action_server.start();
    }


    void executeCommand(const motion_msgs::MovingCommandGoalConstPtr &goal) {
        geometry_msgs::PointStamped goal_point(goal->point_stamped);
        switch (goal->command) {
            case motion_msgs::MovingCommandGoal::MOVE_STANDARD_POSE :
                ROS_INFO("Moving to initial pose.");
                both_arms.setNamedTarget("arms_initial");
                error_code = both_arms.move();
                break;
            case motion_msgs::MovingCommandGoal::MOVE_RIGHT_ARM:
                ROS_INFO("Moving right arm to goal.");
                error_code = moveGroupToCoordinates(right_arm_group, goal_point);
                break;
            case motion_msgs::MovingCommandGoal::MOVE_LEFT_ARM:
                ROS_INFO("Moving left arm to goal.");
                error_code = moveGroupToCoordinates(left_arm_group, goal_point);
                break;
            default:
                ROS_ERROR("COMMAND UNKNOWN");
                result.successful = false;
                action_server.setAborted(result, "UNKNOWN COMMAND. ABORTED.");
                return;
        }

        handleErrorAndReturnResult();
    }

    void handleErrorAndReturnResult() {
        if (error_code.val == error_code.SUCCESS) {
            result.successful = true;
            action_server.setSucceeded(result);
            ROS_INFO("MOVE SUCCESSFUL");
        } else {
            result.successful = false;

            std::string error_string;
            std::ostringstream convert;

            convert << error_code.val;

            error_string = convert.str();

            action_server.setAborted(result, error_string);
            ROS_WARN("%s", error_string.c_str());
        }
    }

    moveit_msgs::MoveItErrorCodes
    moveGroupToCoordinates(moveit::planning_interface::MoveGroup &group, const geometry_msgs::PointStamped &goal_point) {
        geometry_msgs::PointStamped point;
        listener.transformPoint(group.getPlanningFrame(), goal_point, point);
        //geometry_msgs::PoseStamped poseStamped;
        //poseStamped.pose.position.x = goal_point.point.x;
        //poseStamped.pose.position.y = goal_point.point.y;
        //poseStamped.pose.position.z = goal_point.point.z;
        //poseStamped.pose.orientation.w = 1.0;
        //group.setPoseTarget(poseStamped);
        group.setPositionTarget(point.point.x, point.point.y, point.point.z);
        return group.move();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_main");
    ros::NodeHandle node_handle;

    Main main(node_handle);
    ros::spin();

    return 0;
}


