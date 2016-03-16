#include <dynamixel_cpp/dynamixel_device.h>
#include <dynamixel_cpp/dynamixel.h>
#include <ros/console.h>
#include <algorithm>

// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46
#define P_TORQUE_ENABLE		24
#define DXL_CW_COMPLIANCE_SLOPE 28
#define DXL_CCW_COMPLIANCE_SLOPE 29
#define DXL_MAX_TORQUE_L 14
#define DXL_MAX_TORQUE_H 15
#define DXL_TORQUE_LIMIT_L 34
#define DXL_TORQUE_LIMIT_H 35

// User setting
#define BAUD_NUM                1      // 1: 1Mbps 34:57142bps

static const double deg_to_rad = 0.0174532925; // pi / 180
static const double enc_to_deg = 0.29; // RX64 and RX28 documentation
static const double enc_to_rad = enc_to_deg*deg_to_rad;
static const double rad_to_enc = 1.0/enc_to_rad;

void PrintErrorCode();
void printCommStatus(int CommStatus);

DynamixelDevice::DynamixelDevice()
  : torque_enabled(true)
  , initialized(false)
{
}

DynamixelDevice::~DynamixelDevice()
{
  enableTorque(false);
  dxl_terminate();
}

bool DynamixelDevice::init(int feedback_filter_size)
{
  ///////// Open USB2Dynamixel ////////////
  //    if(dxl_initialize(deviceIndex, baudnum) == 0)
  if(dxl_initialize(0, 1) == 0)
  {
    ROS_FATAL("Failed to open USB2Dynamixel!\n");
    return false;
  }

  feedback_buff.clear();
  feedback_buff.resize(motor_ids.size());
  for(size_t i=0; i<motor_ids.size(); ++i)
    feedback_buff[i].set_capacity(feedback_filter_size);

  initialized = true;
  return true;
}

void DynamixelDevice::registerMotor(int motor_id, double* ref, double* act)
{
  if(initialized)
  {
    ROS_ERROR("Cannot register once the device has been initialized!");
    return;
  }

  motor_ids.push_back(motor_id);
  refs.push_back(ref);
  acts.push_back(act);
}

void DynamixelDevice::update()
{
  if(!initialized)
  {
    ROS_ERROR("Update cannot run before initialization!");
    return;
  }

  for(size_t i=0; i<motor_ids.size(); ++i)
  {
    // Read present position
    act_buff = (dxl_read_word(motor_ids[i], P_PRESENT_POSITION_L) - 512) * enc_to_rad;
    comm_status = dxl_get_result();
    // If the reading went wrong dont update actuator position, next iteration we may be luckier
    if(comm_status == COMM_RXSUCCESS)
    {
      *(acts[i]) = act_buff;
    }
    else
    {
      ROS_WARN_STREAM("DynamixelDevice comm error on update of control loop (reading position), motor ID: "
                       << motor_ids[i]);
    }

    // Write goal position
    dxl_write_word(motor_ids[i], P_GOAL_POSITION_L, int((*(refs[i]))*rad_to_enc)+512);
    comm_status = dxl_get_result();
    // Writing a value also has a reading after to know if the writting went ok, so checking
    // that value is suficient
    if(comm_status != COMM_RXSUCCESS)
    {
      //ROS_ERROR_STREAM_THROTTLE(1.0,
      ROS_WARN_STREAM("DynamixelDevice comm error on update of control loop (writing position), motor ID: "
                       << motor_ids[i]);
      printCommStatus(comm_status);
    }
  }
}

void DynamixelDevice::enableTorque(bool enable)
{
  if(torque_enabled != enable)
  {
    for(size_t i=0; i<motor_ids.size(); ++i)
    {
      dxl_write_word(motor_ids[i], P_TORQUE_ENABLE, 0);
    }
    torque_enabled = enable;
  }
}

bool DynamixelDevice::setComplianceSlope(int motor_id, int value)
{
  if(std::find(motor_ids.begin(), motor_ids.end(), motor_id) == motor_ids.end())
  {
    ROS_ERROR_STREAM("Will not set compliance slope for motor with id "
                     << motor_id <<  " since it is not registered");
    return false;
  }

  dxl_write_byte(motor_id, DXL_CW_COMPLIANCE_SLOPE, value);
  dxl_write_byte(motor_id, DXL_CCW_COMPLIANCE_SLOPE, value);
  comm_status = dxl_get_result();
  if(comm_status != COMM_RXSUCCESS)
  {
    ROS_ERROR("Failed to set motor compliance slopes!");
    printCommStatus(comm_status);
    return false;
  }
  return true;
}

int DynamixelDevice::getComplianceSlope(int motor_id)
{
  if(std::find(motor_ids.begin(), motor_ids.end(), motor_id) == motor_ids.end())
  {
    ROS_ERROR_STREAM("Cannot get compliance slope for motor with id "
                     << motor_id <<  " since it is not registered");
    return -1;
  }

  int slope = dxl_read_byte(motor_id, DXL_CW_COMPLIANCE_SLOPE);
  //ROS_ERROR_STREAM_THROTTLE(1.0, "CW slope on motor " << motor_id << ": " << slope);
  int slope2 = dxl_read_byte(motor_id, DXL_CCW_COMPLIANCE_SLOPE);
  //ROS_ERROR_STREAM_THROTTLE(1.0, "CCW slope on motor " << motor_id << ": " << slope);

  return (slope+slope2)/2;
}

double DynamixelDevice::getTorqueLimit(int motor_id)
{
  if(std::find(motor_ids.begin(), motor_ids.end(), motor_id) == motor_ids.end())
  {
    ROS_ERROR_STREAM("Cannot get torque limit for motor with id "
                     << motor_id <<  " since it is not registered");
    return -1;
  }

  int torque_limit_l = dxl_read_word(motor_id, DXL_TORQUE_LIMIT_L);
  ROS_DEBUG_STREAM("Torque limit on motor " << motor_id << " is: " << torque_limit_l << "/" << 0x3FF);
  // In range 0.0 - 1.0
  double normalized_torque = (double) torque_limit_l / (double) 0x3FF;
  ROS_DEBUG_STREAM("Torque limit in 0.0-1.0 range on motor " << motor_id << " is: " << normalized_torque);
  return normalized_torque;
}

bool DynamixelDevice::setTorqueLimit(int motor_id, double limit)
{
  if(std::find(motor_ids.begin(), motor_ids.end(), motor_id) == motor_ids.end())
  {
    ROS_ERROR_STREAM("Cannot set torque limit for motor with id "
                     << motor_id <<  " since it is not registered");
    return false;
  }
  ROS_DEBUG_STREAM("Setting torque limit on motor " << motor_id << ": " << limit);
  int dxl_scale_limit = (int) (limit * (double) 0x3FF);
  ROS_DEBUG_STREAM("Setting dxl_scale_limit: " << dxl_scale_limit << "/" << 0x3FF);
  dxl_write_word(motor_id, DXL_TORQUE_LIMIT_L, dxl_scale_limit);
  comm_status = dxl_get_result();
  if(comm_status != COMM_RXSUCCESS)
  {
    ROS_ERROR("Failed to set torque limit!");
    printCommStatus(comm_status);
    return false;
  }
  return true;
}


void printCommStatus(int CommStatus)
{
  switch(CommStatus)
  {
    case COMM_TXFAIL:
      ROS_DEBUG_STREAM_THROTTLE(1.0,"COMM_TXFAIL: Failed transmit instruction packet!");
      break;

    case COMM_TXERROR:
      ROS_DEBUG_STREAM_THROTTLE(1.0,"COMM_TXERROR: Incorrect instruction packet!");
      break;

    case COMM_RXFAIL:
      ROS_DEBUG_STREAM_THROTTLE(1.0,"COMM_RXFAIL: Failed get status packet from device!");
      break;

    case COMM_RXWAITING:
      ROS_DEBUG_STREAM_THROTTLE(1.0,"COMM_RXWAITING: Now recieving status packet!");
      break;

    case COMM_RXTIMEOUT:
      ROS_DEBUG_STREAM_THROTTLE(1.0,"COMM_RXTIMEOUT: There is no status packet!");
      break;

    case COMM_RXCORRUPT:
      ROS_DEBUG_STREAM_THROTTLE(1.0,"COMM_RXCORRUPT: Incorrect status packet!");
      break;

    default:
      ROS_DEBUG_STREAM_THROTTLE(1.0,"This is unknown error code " << CommStatus);
      break;
  }
}

// Print error bit of status packet
void PrintErrorCode()
{
  if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Input voltage error!");

  if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Angle limit error!");

  if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Overheat error!");

  if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Out of range error!");

  if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Checksum error!");

  if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Overload error!");

  if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Instruction code error!");
}
