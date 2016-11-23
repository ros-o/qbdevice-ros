#include <qb_device_hardware_interface/qb_device_hardware_interface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "qb_device_hardware_interface");

  qb_device_hardware_interface::qbDeviceHW qb_device;

  return 0;
}