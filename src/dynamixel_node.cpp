#include <ros/ros.h>
#include <dynamixel_cpp/dynamixel_device.h>
#include <dynamic_reconfigure/server.h>
#include <dynamixel_cpp/DynControlConfig.h>
#include <sensor_msgs/JointState.h>

double ref[] = {0.0,0.0};

void reconfCallback(dynamixel_cpp::DynControlConfig &config, uint32_t level) {
  ref[0] = config.motor_1;
  ref[1] = config.motor_2;
}

void refCallback(const sensor_msgs::JointStateConstPtr& ref_state)
{
  ref[0] = ref_state->position[0];
  ref[1] = ref_state->position[1];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_node");
  ros::NodeHandle nh;
  ros::Rate rate(20);
  dynamic_reconfigure::Server<dynamixel_cpp::DynControlConfig> server;
  dynamic_reconfigure::Server<dynamixel_cpp::DynControlConfig>::CallbackType f;

  ros::Subscriber state_sub = nh.subscribe("dynamixel_ref", 50, &refCallback);
  ros::Publisher state_pub = nh.advertise<sensor_msgs::JointState>("dynamixel_act", 50);
  sensor_msgs::JointState act_state;
  act_state.position.resize(2);
  act_state.velocity.resize(2);
  act_state.effort.resize(2);

  f = boost::bind(&reconfCallback, _1, _2);
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
//    for(int i=0; i<2; ++i)
//    {
//      ROS_INFO_STREAM("Act: " << act[i] <<
//                      " , ref:" << ref[i]);
//    }
    dxl.update();
    act_state.position[0] = act[0];
    act_state.position[1] = act[1];
    act_state.header.stamp = ros::Time::now();
    state_pub.publish(act_state);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
