#ifndef QB_DEVICE_HARDWARE_INTERFACE_H
#define QB_DEVICE_HARDWARE_INTERFACE_H

// ROS libraries
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>

// qb robotics libraries
#include <qb_device_driver/qb_device_driver.h>

namespace qb_device_hardware_interface {
class qbDeviceHW : public hardware_interface::RobotHW {
 public:
  qbDeviceHW();
  virtual ~qbDeviceHW();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

 private:
  ros::NodeHandle private_node_handler_;
};
}  // namespace qb_device_hardware_interface

#endif // QB_DEVICE_HARDWARE_INTERFACE_H