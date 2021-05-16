#include <ros/ros.h>
#include <alex_global/global_definitions.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <alex_kinematics/alex_ikine.h>


void circlePath(double, double, double, geometry_msgs::TransformStamped&);

int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "alex_ikine_test_node");
  ros::NodeHandle node;

  //ros::Publisher jointStatePub = node.advertise<sensor_msgs::JointState>("alex_ikine_joint_states", 1000);
  alex_kinematics::alex_ikine ikineSrv;
  ros::ServiceClient ikineClient = node.serviceClient<alex_kinematics::alex_ikine>("alex_ikine");
  geometry_msgs::TransformStamped target_left_foot_a;
  // target_left_foot_a.child_frame_id = "right_foot_a";
  // target_left_foot_a.transform.translation.x = -0.1;
  // //target_left_foot_a.transform.translation.y = 0.2;
  // target_left_foot_a.transform.translation.z = -0.4;
  //
  // ros::Rate r(60);
  // int circleIndex = 0;
  // int circleSegments = 200;
  // double circleRadius = 0.05;
  while (ros::ok()) {
    ikineSrv.request.footTransforms.clear();
    sensor_msgs::JointState jointStates;
    target_left_foot_a.child_frame_id = "TEST";
    target_left_foot_a.transform.translation.x = -0.001479;
    target_left_foot_a.transform.translation.y = 0.098158;
    target_left_foot_a.transform.translation.z = -0.509519;

    ikineSrv.request.footTransforms.push_back(target_left_foot_a);
    ikineClient.call(ikineSrv);

    //jointStates.name = ikineSrv.response.jointStates.name;
    //jointStates.position = ikineSrv.response.jointStates.position;

    // jointStatePub.publish(ikineSrv.response.jointStates);


    ros::spinOnce();
    //r.sleep();
  }
  return 0;
}

void circlePath(double circleSegments, double segmentIndex, double circleRadius, geometry_msgs::TransformStamped& footTransform) {
  double centreX = -0.1;
  double centreZ = -0.4;
  double segment = 2*M_PI / circleSegments;
  double currentAngle = segment * segmentIndex;
  footTransform.transform.translation.x = centreX + circleRadius * cos(currentAngle);
  footTransform.transform.translation.z = centreZ + circleRadius * sin(currentAngle);
  // std::cout << "CIRCLE X: " << footTransform.transform.translation.x << std::endl;
  // std::cout << "CIRCLE Z: " << footTransform.transform.translation.z << std::endl;

}
