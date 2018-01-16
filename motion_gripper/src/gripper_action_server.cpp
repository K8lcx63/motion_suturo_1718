#include <ros/ros.h>
#include <motion_msgs/GripperAction.h>
#include <actionlib/server/simple_action_server.h>
#include "Gripper.cpp"

const std::string right_gripper_controller_name = "r_gripper_controller/gripper_action";
const std::string left_gripper_controller_name = "l_gripper_controller/gripper_action";

class GripperActionServer {
private:
    ros::NodeHandle node_handle;
    actionlib::SimpleActionServer<motion_msgs::GripperAction> action_server;
public:
    GripperActionServer(const ros::NodeHandle &nh) :
            node_handle(nh),
            action_server(node_handle, "gripper", boost::bind(&GripperActionServer::executeCommand, this, _1), false) {
        action_server.start();
    };

    void executeCommand(const motion_msgs::GripperGoalConstPtr &goal) {
        if (goal->gripper == motion_msgs::GripperGoal::LEFT) {
            Gripper gripper(left_gripper_controller_name);
            gripper.moveGripper(goal->position, goal->effort);
        } else if (goal->gripper == motion_msgs::GripperGoal::RIGHT) {
            Gripper gripper(right_gripper_controller_name);
            gripper.moveGripper(goal->position, goal->effort);
        } else {
            ROS_ERROR("UNKNOWN GRIPPER");
        }
    };
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_gripper");
    ros::NodeHandle node_handle;

    GripperActionServer gripperActionServer(node_handle);

    ros::spin();

    return 0;
}
