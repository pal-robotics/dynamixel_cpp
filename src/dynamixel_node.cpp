#include <ros/ros.h>
#include <dynamixel_cpp/dynamixel_device.h>
#include <dynamic_reconfigure/server.h>
#include <dynamixel_cpp/DynControlConfig.h>

double ref[] = {0.0,0.0};

void callback(dynamixel_cpp::DynControlConfig &config, uint32_t level) {
  ref[0] = config.motor_1;
  ref[1] = config.motor_2;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_node");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  dynamic_reconfigure::Server<dynamixel_cpp::DynControlConfig> server;
  dynamic_reconfigure::Server<dynamixel_cpp::DynControlConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  DynamixelDevice dxl;

  double act[] = {0.0,0.0};

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
