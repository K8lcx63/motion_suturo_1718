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

    ~Gripper();

    actionlib::SimpleClientGoalState moveGripper(float position, float effort);

    void open();

    void close();

};

#endif //MOTION_GRIPPER_GRIPPER_H
