#ifndef __DYNAMIXEL_CPP_DYNAMIXEL__DEVICE__
#define __DYNAMIXEL_CPP_DYNAMIXEL__DEVICE__

#include <vector>
#include <boost/circular_buffer.hpp>

class DynamixelDevice
{
public:
  DynamixelDevice();
  virtual ~DynamixelDevice();

  bool init(int feedback_filter_size=5);

  void registerMotor(int motor_id, double* ref, double* act);

  void update();

  void enableTorque(bool enable);

  void setComplianceSlope(int motor_id, int value);
  int getComplianceSlope(int motor_id);

private:
  bool initialized;

  std::vector<int> motor_ids;
  std::vector<double*> refs;
  std::vector<double*> acts;

  double act_buff;

  int comm_status;

  bool torque_enabled;

  std::vector<boost::circular_buffer<double> > feedback_buff;
};
#endif
