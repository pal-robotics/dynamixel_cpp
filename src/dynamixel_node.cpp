#include <dynamixel_cpp/dynamixel_device.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_node");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  DynamixelDevice dxl;

  double act[] = {0.0,0.0};
  double ref[] = {0.0,0.0};

  dxl.registerMotor(1, &ref[0], &act[0]);
  dxl.registerMotor(3, &ref[1], &act[1]);
  ROS_INFO("Registered motors");

  dxl.init();
  ROS_INFO("Dynamixel initialized");

  while(ros::ok())
  {
    for(int i=0; i<2; ++i)
    {
      ROS_INFO_STREAM("Act: " << act[i] <<
                      " , ref:" << ref[i]);
    }
    dxl.update();
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
