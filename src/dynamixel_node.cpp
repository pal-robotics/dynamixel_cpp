#include <ros/ros.h>
#include <dynamixel_cpp/dynamixel_device.h>
#include <sensor_msgs/JointState.h>


class DynamixelMotor
{
public:
  std::string name_;
  int id_;
  int compliance_slope_;
  double torque_limit_;
  double ref;
  double act;
  // in case a dynamixel is mounted the other way around we can just invert the commands
  bool invert_commands;

  DynamixelMotor(std::string name, int id, int compliance_slope = 0, double torque_limit = 1.0, bool invert_cmds = false)
    : name_(name)
    , id_(id)
    , compliance_slope_(compliance_slope)
    , torque_limit_(torque_limit)
    , invert_commands(invert_cmds)
  {}
};

std::vector<DynamixelMotor> motors;
DynamixelDevice dxl;

void refCallback(const sensor_msgs::JointStateConstPtr& ref_state)
{
  for(size_t i=0; i<motors.size(); ++i)
  {
    if (motors[i].invert_commands)
      motors[i].ref = ref_state->position[i] * -1.0;
    else
      motors[i].ref = ref_state->position[i];
  }
}

void timerComplianceCallback(const ros::TimerEvent&)
{
  // Perform a check on the compliance with a configured rate and fix if needed
  for(size_t i=0; i<motors.size(); ++i){
    int curr_cs = dxl.getComplianceSlope(motors[i].id_);
    if(curr_cs != motors[i].compliance_slope_)
    {
      // It's a bad read, not that it's wrong set usually :/
      ROS_WARN_STREAM("Compliance slope for motor ID" << motors[i].id_ << " should be " << motors[i].compliance_slope_ << " but is " << curr_cs);
      //dxl.setComplianceSlope(motors[i].id_, motors[i].compliance_slope_);
    }
  }
}

void timerTorqueCallback(const ros::TimerEvent&)
{
  // Perform a check on the compliance with a configured rate and fix if needed
  for(size_t i=0; i<motors.size(); ++i){
    double curr_tl = dxl.getTorqueLimit(motors[i].id_);
    if(std::abs(curr_tl - motors[i].torque_limit_) > 0.005)
    {
      // double check...
      curr_tl = dxl.getTorqueLimit(motors[i].id_);
      if(std::abs(curr_tl - motors[i].torque_limit_) > 0.005)
      {
        // It's a bad read, not that it's wrong set usually :/
        ROS_WARN_STREAM("Torque limit for motor ID" << motors[i].id_ << " should be " << motors[i].torque_limit_ << " but is " << curr_tl);
        dxl.setTorqueLimit(motors[i].id_, motors[i].torque_limit_);
      }

    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_node");
  ros::NodeHandle nh;
  ros::Rate rate(20);


  if(nh.hasParam("dynamixel_parameters"))
  {
    ROS_WARN_STREAM("Has parameters");
    XmlRpc::XmlRpcValue filters;
    nh.getParam("dynamixel_parameters", filters);
    ROS_WARN_STREAM("Got " << filters.size() << " parameters");
    ROS_ASSERT(filters.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = filters.begin(); it != filters.end(); ++it)
    {
      const std::string actuator_name = it->first;
      const int         id            = filters[it->first]["id"];
      const int      compliance_slope = filters[it->first]["compliance_slope"];
      const double    torque_limit    = filters[it->first]["torque_limit"];
      const bool      invert_cmd      = filters[it->first]["invert_commands"];
      motors.push_back(DynamixelMotor(actuator_name, id, compliance_slope, torque_limit, invert_cmd));
    }
  }
  else
  {
    ROS_ERROR("There is no dynamixel_parameters on the param server, stopping...");
    return 1;
  }

  ros::Subscriber state_sub = nh.subscribe("dynamixel_ref", 50, &refCallback);
  ros::Publisher state_pub = nh.advertise<sensor_msgs::JointState>("dynamixel_act", 50);

  // TODO: make all these hardcoded values parameters
  sensor_msgs::JointState act_state;
  act_state.position.resize(motors.size());
  act_state.velocity.resize(motors.size());
  act_state.effort.resize(motors.size());

  for(size_t i=0; i<motors.size(); ++i)
  {
    act_state.name.push_back(motors[i].name_);
    dxl.registerMotor(motors[i].id_, &motors[i].ref, &motors[i].act);
  }
  ROS_INFO("Registered motors");

  dxl.init();
  ROS_INFO("Dynamixel initialized");

  for(size_t i=0; i<motors.size(); ++i)
  {
    while(! dxl.setComplianceSlope(motors[i].id_, motors[i].compliance_slope_))
    {
      ROS_ERROR_STREAM("Retrying setting compliance slope to motor id: " << motors[i].id_ << " to " << motors[i].compliance_slope_);
    }
  }
  for(size_t i=0; i<motors.size(); ++i)
  {
    while(! dxl.setTorqueLimit(motors[i].id_, motors[i].torque_limit_))
    {
     ROS_ERROR_STREAM("Retrying setting torque limit to motor id: " << motors[i].id_ << " to " << motors[i].torque_limit_);
    }
  }

  // This should not be needed, Bence said it was necessary on Marco... to be tested.
  // ros::Timer compliance_checker = nh.createTimer(ros::Duration(3.0), &timerComplianceCallback);
  ros::Timer torque_checker = nh.createTimer(ros::Duration(3.0), &timerTorqueCallback);

  while(ros::ok())
  {
    // Send new and get act positions
    dxl.update();

    // Update feedback and publish
    act_state.header.stamp = ros::Time::now();
    for(size_t i=0; i<motors.size(); ++i)
    {
      if (motors[i].invert_commands)
        act_state.position[i] = motors[i].act * -1.0;
      else
        act_state.position[i] = motors[i].act;
    }
    state_pub.publish(act_state);

    // Update ROS and sleep a bit
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
