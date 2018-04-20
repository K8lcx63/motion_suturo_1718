#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

#ifndef MOTION_GRIPPER_GRIPPER_H
#define MOTION_GRIPPER_GRIPPER_H

typedef actionlib::SimpleActionClient <pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper {
private:
    GripperClient *gripper_client_;
public:
    /**
     * Constructor.
     * @param actionName
     */
    Gripper(const std::string actionName);

    /**
     * Destructor.
     */
    ~Gripper();

    /**
     * Moves the gripper to the given position with the given effort.
     * @param position position to move the gripper to.
     * @param effort effort when closing the gripper.
     * @return The result goal state.
     */
    actionlib::SimpleClientGoalState moveGripper(float position, float effort);

    /**
     * Opens the gripper.
     */
    void open();

    /**
     * Closes the gripper with maximum effort.
     */
    void close();

    static pr2_controllers_msgs::Pr2GripperCommandGoal generateGripperGoal(float position, float effort) {
        pr2_controllers_msgs::Pr2GripperCommandGoal goal;
        goal.command.position = position;
        goal.command.max_effort = effort;
        return goal;
    }
};

#endif //MOTION_GRIPPER_GRIPPER_H
