#include <dynamixel_cpp/dynamixel_device.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_node");
  ros::NodeHandle nh;

  DynamixelDevice dxl;

  double act = 0.0;
  double ref = 0.0;

  dxl.registerMotor(2, &ref, &act);
  dxl.registerMotor(3, &ref, &act);
  ROS_INFO("Registered motors");

  dxl.init();
  ROS_INFO("Dynamixel initialized");

  while(ros::ok())
  {
      ROS_INFO_STREAM_THROTTLE(0.3, "Act: " << act << " , ref:" << ref);
      dxl.update();
      ros::spinOnce();
  }

  return 0;
}
