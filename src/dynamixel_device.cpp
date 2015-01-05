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

void DynamixelDevice::init()
{
  ///////// Open USB2Dynamixel ////////////
  //    if(dxl_initialize(deviceIndex, baudnum) == 0)
  if(dxl_initialize(0, 1) == 0)
  {
    ROS_FATAL("Failed to open USB2Dynamixel!\n");
  }
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
    *(acts[i]) = (dxl_read_word(motor_ids[i], P_PRESENT_POSITION_L) - 512) * enc_to_rad;

    // Write goal position
    dxl_write_word(motor_ids[i], P_GOAL_POSITION_L, int((*(refs[i])*rad_to_enc)+512));

    comm_status = dxl_get_result();
    if(comm_status != COMM_RXSUCCESS)
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
    printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
    break;

  case COMM_TXERROR:
    printf("COMM_TXERROR: Incorrect instruction packet!\n");
    break;

  case COMM_RXFAIL:
    printf("COMM_RXFAIL: Failed get status packet from device!\n");
    break;

  case COMM_RXWAITING:
    printf("COMM_RXWAITING: Now recieving status packet!\n");
    break;

  case COMM_RXTIMEOUT:
    printf("COMM_RXTIMEOUT: There is no status packet!\n");
    break;

  case COMM_RXCORRUPT:
    printf("COMM_RXCORRUPT: Incorrect status packet!\n");
    break;

  default:
    printf("This is unknown error code %d \n", CommStatus);
    break;
  }
}

// Print error bit of status packet
void PrintErrorCode()
{
  if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    printf("Input voltage error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    printf("Angle limit error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    printf("Overheat error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    printf("Out of range error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    printf("Checksum error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    printf("Overload error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    printf("Instruction code error!\n");
}
