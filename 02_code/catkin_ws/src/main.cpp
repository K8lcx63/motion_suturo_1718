#include <ros/node_handle.h>
#include <ros/ros.h>

class Main {
private:
    ros::NodeHandle node_handle;

public:
    Main(const ros::NodeHandle &nh) :
            node_handle(nh)
            {

    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "main_motion");
        ros::NodeHandle nh;
        Main main(nh);
        ros::spin();
    }
};