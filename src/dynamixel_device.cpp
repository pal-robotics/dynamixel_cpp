#include <dynamixel_cpp/dynamixel_device.h>
#include <dynamixel_cpp/dynamixel.h>
#include <ros/console.h>

// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46
#define P_TORQUE_ENABLE		24

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

  feedback_buff.set_capacity(feedback_filter_size);
  return true;
}

void DynamixelDevice::registerMotor(int motor_id, double* ref, double* act)
{
  motor_ids.push_back(motor_id);
  refs.push_back(ref);
  acts.push_back(act);
}

void DynamixelDevice::update()
{
    for(size_t i=0; i<motor_ids.size(); ++i)
    {
      // Read present position
      act_buff = (dxl_read_word(motor_ids[i], P_PRESENT_POSITION_L) - 512) * enc_to_rad;

      // Write goal position
      dxl_write_word(motor_ids[i], P_GOAL_POSITION_L, int((*(refs[i]))*rad_to_enc)+512);

      comm_status = dxl_get_result();
      if(comm_status == COMM_RXSUCCESS)
      {
//        if(act_buff == 0 && fabs(*(acts[i])) > 0.01)
//        {
//           ROS_ERROR("ZERO WTF");
//           break;
//        }
        // result is median of last N elements (if have enough elements)
        if(feedback_buff.size() == feedback_buff.capacity())
        {
          feedback_buff.push_back(act_buff);
          std::nth_element(feedback_buff.begin(),
                           feedback_buff.begin() + feedback_buff.size()/2,
                           feedback_buff.end());
          *(acts[i]) = feedback_buff[feedback_buff.size() / 2];
        }
        else
        {
          *(acts[i]) = act_buff;
        }
      }
      else
      {
        ROS_ERROR_STREAM_THROTTLE(1.0,
                                  "DynamixelDevice comm error, motor ID: "
                                  << motor_ids[i]);
        printCommStatus(comm_status);
        break;
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

void printCommStatus(int CommStatus)
{
  switch(CommStatus)
  {
  case COMM_TXFAIL:
    ROS_ERROR_STREAM_THROTTLE(1.0,"COMM_TXFAIL: Failed transmit instruction packet!");
    break;

  case COMM_TXERROR:
    ROS_ERROR_STREAM_THROTTLE(1.0,"COMM_TXERROR: Incorrect instruction packet!");
    break;

  case COMM_RXFAIL:
    ROS_ERROR_STREAM_THROTTLE(1.0,"COMM_RXFAIL: Failed get status packet from device!");
    break;

  case COMM_RXWAITING:
    ROS_ERROR_STREAM_THROTTLE(1.0,"COMM_RXWAITING: Now recieving status packet!");
    break;

  case COMM_RXTIMEOUT:
    ROS_ERROR_STREAM_THROTTLE(1.0,"COMM_RXTIMEOUT: There is no status packet!");
    break;

  case COMM_RXCORRUPT:
    ROS_ERROR_STREAM_THROTTLE(1.0,"COMM_RXCORRUPT: Incorrect status packet!");
    break;

  default:
    ROS_ERROR_STREAM_THROTTLE(1.0,"This is unknown error code " << CommStatus);
    break;
  }
}

// Print error bit of status packet
void PrintErrorCode()
{
  if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    ROS_ERROR_STREAM_THROTTLE(1.0,"Input voltage error!");

  if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    ROS_ERROR_STREAM_THROTTLE(1.0,"Angle limit error!");

  if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    ROS_ERROR_STREAM_THROTTLE(1.0,"Overheat error!");

  if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    ROS_ERROR_STREAM_THROTTLE(1.0,"Out of range error!");

  if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    ROS_ERROR_STREAM_THROTTLE(1.0,"Checksum error!");

  if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    ROS_ERROR_STREAM_THROTTLE(1.0,"Overload error!");

  if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    ROS_ERROR_STREAM_THROTTLE(1.0,"Instruction code error!");
}
