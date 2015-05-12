#include <ros/ros.h>
#include <dynamixel_cpp/dynamixel_device.h>
#include <dynamic_reconfigure/server.h>
#include <dynamixel_cpp/DynControlConfig.h>
#include <sensor_msgs/JointState.h>

double ref[] = {0.0,0.0};
int compliance_slope[] = {50, 50};
DynamixelDevice dxl;

void reconfCallback(dynamixel_cpp::DynControlConfig &config, uint32_t level) {
  ref[0] = config.motor_1;
  ref[1] = config.motor_2;
}

void refCallback(const sensor_msgs::JointStateConstPtr& ref_state)
{
  ref[0] = ref_state->position[0];
  ref[1] = ref_state->position[1];
}

void timerCallback(const ros::TimerEvent&)
{
  // Perform a check on the compliance with a configured rate and fix if needed
  if(dxl.getComplianceSlope(1) != compliance_slope[0])
    dxl.setComplianceSlope(1, compliance_slope[0]);
  if(dxl.getComplianceSlope(3) != compliance_slope[1])
    dxl.setComplianceSlope(3, compliance_slope[1]);
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

  // TODO: make all these hardcoded values parameters
  sensor_msgs::JointState act_state;
  act_state.position.resize(2);
  act_state.velocity.resize(2);
  act_state.effort.resize(2);

  f = boost::bind(&reconfCallback, _1, _2);
  server.setCallback(f);


  double act[] = {0.0,0.0};

  dxl.registerMotor(1, &ref[0], &act[0]);
  dxl.registerMotor(3, &ref[1], &act[1]);
  ROS_INFO("Registered motors");

  dxl.init();
  ROS_INFO("Dynamixel initialized");

  dxl.setComplianceSlope(1, compliance_slope[0]);
  dxl.setComplianceSlope(3, compliance_slope[1]);
  ros::Timer compliance_checker = nh.createTimer(ros::Duration(3.0), &timerCallback);

  while(ros::ok())
  {
    // Send new and get act positions
    dxl.update();

    // Update feedback and publish
    act_state.header.stamp = ros::Time::now();
    act_state.position[0] = act[0];
    act_state.position[1] = act[1];
    state_pub.publish(act_state);

    // Update ROS and sleep a bit
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
