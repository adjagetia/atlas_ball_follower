#include <tough_controller_interface/chest_control_interface.h>

int main(int argc, char** argv)
{
    // init ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); 
    spinner.start(); // ensure ROS callbacks are activated

    // create control obj
    ChestControlInterface chestTraj(nh);

    float roll = 0.0f * M_PI / 180.0f;
    float pitch = 10.0f * M_PI / 180.0f;
    float yaw = 10.0f * M_PI / 180.0f;
    float duration = 2.0f;
    
    // change the orientation (note this is non-blocking)
    chestTraj.controlChest(roll, pitch, yaw, duration);

    ros::Duration(duration).sleep();
}