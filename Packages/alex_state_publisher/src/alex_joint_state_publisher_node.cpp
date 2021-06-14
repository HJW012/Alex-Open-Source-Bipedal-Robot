#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "alex_joint_state_publisher");


  ros::NodeHandle n;

  ros::Publisher joint_State_pub = n.advertise<sensor_msgs::JointState>("alex_current_joint_states", 1000);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    sensor_msgs::JointState msg;
    joint_State_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

// False if joint states are old
bool get_current_joint_states() {

}
