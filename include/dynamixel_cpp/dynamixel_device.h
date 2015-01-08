#ifndef __DYNAMIXEL_CPP_DYNAMIXEL__DEVICE__
#define __DYNAMIXEL_CPP_DYNAMIXEL__DEVICE__

#include <vector>

class DynamixelDevice
{
public:
  DynamixelDevice();
  virtual ~DynamixelDevice();

  bool init();

  void registerMotor(int motor_id, double* ref, double* act);

  void update();
  void updateLoop();

  void enableTorque(bool enable);

private:
  std::vector<int> motor_ids;
  std::vector<double*> refs;
  std::vector<double*> acts;

  int comm_status;

  bool torque_enabled;
};
#endif
