#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <tough_controller_interface/chest_control_interface.h>

ros::Publisher chatter_pub;
ros::Subscriber sub;
int count;

/**
 * callback function - called on heard message
**/
void callback(const std_msgs::Float32::ConstPtr& msg)
{
  ros::NodeHandle n;
  chatter_pub = n.advertise<std_msgs::Float32>("result", 1000);

  ROS_INFO("I heard: [%f]", msg->data);

  
  ChestControlInterface chestTraj(n);
  float roll = 0.0f * M_PI / 180.0f;
  float pitch = 0.0f * M_PI / 180.0f;
  float yaw = msg->data * M_PI / 180.0f;
  float duration = 2.0f;
  // change the orientation (note this is non-blocking)
  chestTraj.controlChest(roll, pitch, yaw, duration);
  ros::Duration(duration).sleep();

  std_msgs::String msg2;

  std::stringstream ss;
  ss << "Psuedo Done!";
  msg2.data = ss.str();

  ROS_INFO("%s", msg2.data.c_str());

  chatter_pub.publish(msg2);
}

/**
 * main function - called on entry to node
**/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "chest_node");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1); 
  spinner.start(); // ensure ROS callbacks are activated

  count = 0;
  
  sub = n.subscribe("values", 1000, callback);
  
  ros::spin();
  return 0;
}