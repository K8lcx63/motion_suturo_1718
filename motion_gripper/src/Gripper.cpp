#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient <pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper {
private:
    GripperClient *gripper_client_;
public:
    Gripper(const std::string actionName) {
        gripper_client_ = new GripperClient(actionName, true);
        ROS_INFO("Connecting to Gripper Client - %s", actionName.c_str());
        while (!gripper_client_->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the %s action server to come up", actionName.c_str());
        }
    }

    ~Gripper() {
        delete gripper_client_;
    }

    actionlib::SimpleClientGoalState moveGripper(float position, float effort) {
        pr2_controllers_msgs::Pr2GripperCommandGoal goal;
        goal.command.position = position;
        goal.command.max_effort = effort;
        ROS_INFO("Sending gripper goal");
        gripper_client_->sendGoal(goal);
        gripper_client_->waitForResult();
        return gripper_client_->getState();
    }

    void open() {
        pr2_controllers_msgs::Pr2GripperCommandGoal open;
        open.command.position = 0.07;
        open.command.max_effort = -1;  // Do not limit effort (negative)
        ROS_INFO("Sending open goal");
        gripper_client_->sendGoal(open);
        gripper_client_->waitForResult();
        if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The gripper opened!");
        } else {
            ROS_INFO("The gripper failed to open.");
            //ROS_INFO(gripper_client_->getState().toString().c_str());
            if(gripper_client_->isServerConnected()) {
                ROS_INFO("Client is connected");
            } else {
                ROS_INFO("Client NOT connected");
            }
        }
    }

    void close() {
        pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
        squeeze.command.position = 0.01;
        squeeze.command.max_effort = -1;
        ROS_INFO("Sending squeeze goal");
        gripper_client_->sendGoal(squeeze);
        gripper_client_->waitForResult();
        if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The gripper closed!");
        } else {
            ROS_INFO("The gripper failed to close.");
            //ROS_INFO(gripper_client_->getState().toString().c_str());
            if(gripper_client_->isServerConnected()) {
                ROS_INFO("Client is connected");
            } else {
                ROS_INFO("Client NOT connected");
            }
        }
    }
};

