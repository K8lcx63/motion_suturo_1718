#ifndef MOTION_GRIPPER_GRIPPER_H
#define MOTION_GRIPPER_GRIPPER_H

typedef actionlib::SimpleActionClient <pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper {
private:
    GripperClient *gripper_client_;
public:
    /**
     * Constructor
     * @param actionName
     */
    Gripper(const std::string actionName);

    /**
     * Destructor.
     */
    ~Gripper();

    /**
     * Move Gripper to the given position with the given effort.
     * @param position Position to move Gripper to.
     * @param effort  Effort when closing gripper.
     * @return  SimpleClientGoalState (sucess, failed).
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

};

#endif //MOTION_GRIPPER_GRIPPER_H
