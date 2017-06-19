/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2017, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QB_DEVICE_HARDWARE_INTERFACE_H
#define QB_DEVICE_HARDWARE_INTERFACE_H

// ROS libraries
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/transmission_interface.h>
#include <urdf/model.h>

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_resources.h>
#include <qb_device_hardware_interface/qb_device_joint_limits_interface.h>
#include <qb_device_hardware_interface/qb_device_joint_limits_resources.h>
#include <qb_device_hardware_interface/qb_device_transmission_resources.h>
#include <qb_device_msgs/qb_device_msgs.h>
#include <qb_device_srvs/qb_device_srvs.h>

namespace qb_device_hardware_interface {
/**
 * The qbrobotics Device HardWare interface extends the \p hardware_interface::RobotHW by providing all the common
 * structures to manage the communication with both the \em qbhand and \em qbmove devices. The few differences among
 * them are implemented in the two device-specific derived classes, \p qbHandHW and \p qbMoveHW. In detail, this class
 * initialize all the ROS service clients needed to talk to the Communication Handler which is the intermediary for
 * the serial communication with the physical devices. The core part is given by the two overridden function of the
 * \p hardware_interface::RobotHW base class, i.e. read() and write(), which are meant to be used in the control loop.
 * \sa qb_hand_hardware_interface::qbHandHW, qb_move_hardware_interface::qbMoveHW
 */
class qbDeviceHW : public hardware_interface::RobotHW {
 public:
  /**
   * Initialize HW resources and interfaces, and wait until it is not registered on the Communication Handler. If the
   * Communication Handler is not running, wait forever. The three given parameters are strictly device-dependent, i.e.
   * the \em qbhand specific \p transmission differs from the \em qbmove one, and the actuator and joint names depend
   * not only on the device type, but also on the whole \p robot_description extracted from the URDF model. For example
   * each \em qbmove of a given chain has its own namespace to avoid name clashes.
   * \param transmission The shared pointer to the current transmission derived from \p transmission_interface::Transmission.
   * \param actuators The actuator names in the \p robot_description.
   * \param joints The joint names in the \p robot_description.
   * \sa initializeInterfaces(), initializeResources(), waitForRegistration(), waitForServices()
   */
  qbDeviceHW(qb_device_transmission_interface::TransmissionPtr transmission, const std::vector<std::string> &actuators, const std::vector<std::string> &joints);

  /**
   * Call the service to deregister the device on the Communication Handler.
   * \sa deregisterDevice()
   */
  virtual ~qbDeviceHW();

  /**
   * \return The device ID, in range [\p 1, \p 128].
   */
  int getDeviceId() { return device_.id; }

  /**
   * \return The device namespace used to avoid name clashes among same-type devices.
   */
  std::string getDeviceNamespace() { return namespace_; }

  /**
   * This pure virtual method has to be redefined by derived classes to return the controlled joint names vector.
   * \return The vector of controller joint names.
   */
  virtual std::vector<std::string> getJoints() = 0;

  /**
   * Read actuator state from the hardware, propagate it to joint states and publish the whole device state to a
   * namespaced \p "~state" topic (each instance of qbDeviceHW should publish on its own topic).
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   * \sa getMeasurements(), publish()
   */
  virtual void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Enforce joint limits for all registered joint limit interfaces, propagate joint commands to actuators, and send
   * actuator commands to the hardware.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   * \sa qb_device_joint_limits_interface::qbDeviceJointLimitsResources::enforceLimits(), setCommands()
   */
  virtual void write(const ros::Time& time, const ros::Duration& period);

 protected:
  ros::AsyncSpinner spinner_;
  ros::NodeHandle node_handle_;
  ros::Publisher state_publisher_;
  //TODO: pack all the service in a std::map services_(name, client)
  ros::ServiceClient activate_motors_;
  ros::ServiceClient deactivate_motors_;
  ros::ServiceClient deregister_device_;
  ros::ServiceClient get_info_;
  ros::ServiceClient get_measurements_;
  ros::ServiceClient register_device_;
  ros::ServiceClient set_commands_;
  qb_device_hardware_interface::qbDeviceResources device_;
  qb_device_hardware_interface::qbDeviceHWResources actuators_;
  qb_device_hardware_interface::qbDeviceHWResources joints_;
  qb_device_hardware_interface::qbDeviceHWInterfaces interfaces_;
  qb_device_joint_limits_interface::qbDeviceJointLimitsResources joint_limits_;
  qb_device_transmission_interface::qbDeviceTransmissionResources transmission_;
  urdf::Model urdf_model_;
  std::string namespace_;

  /**
   * Call the service to activate the device motors and wait for the response.
   * \return \p 0 on success.
   * \sa deactivateMotors()
   */
  virtual int activateMotors();

  /**
   * Call the service to deactivate the device motors and wait for the response.
   * \return \p 0 on success.
   * \sa activateMotors()
   */
  virtual int deactivateMotors();

  /**
   * Call the service to deregister the device from the Communication Handler and wait for the response.
   * \return \p 0 on success.
   * \sa registerDevice(), waitForRegistration()
   */
  virtual int deregisterDevice();

  /**
   * Call the service to retrieve the printable configuration setup of the device and wait for the response.
   * \return The configuration setup formatted as a plain text string (empty string on error).
   */
  virtual std::string getInfo();

  /**
   * Call the service to retrieve device measurements (both positions and currents) and wait for the response. When
   * data is received, correct the positions direction with the \p motor_axis_direction.
   * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
   * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
   * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
   * not directly actuated).
   * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
   * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
   * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
   * \return \p 0 on success.
   * \sa setCommands()
   */
  virtual int getMeasurements(std::vector<double> &positions, std::vector<double> &currents);

  /**
   * Initialize the \p hardware_interface, the \p joint_limit_interface, and the \p transmission_interface.
   * \param transmission The shared pointer to the current transmission derived from \p transmission_interface::Transmission.
   * \sa qb_device_hardware_interface::qbDeviceHWInterfaces::initialize(),
   *     qb_device_joint_limits_interface::qbDeviceJointLimitsResources::initialize(),
   *     qb_device_transmission_interface::qbDeviceTransmissionResources::initialize()
   */
  void initializeInterfaces(qb_device_transmission_interface::TransmissionPtr transmission);

  /**
   * Initialize the HW resources and retrieve the \p robot_description from the Parameter Server.
   * \param actuators The actuator names in the \p robot_description.
   * \param joints The joint names in the \p robot_description.
   */
  void initializeResources(const std::vector<std::string> &actuators, const std::vector<std::string> &joints);

  /**
   * Call the service to register the device node to the Communication Handler and wait for the response. If the
   * registration succeed, store the device parameters received, e.g. \p position_limits.
   * \param activate_on_registration If \p true ask for motors activation during the registration process.
   * \return \p 0 on success.
   * \sa deregisterDevice(), waitForRegistration()
   */
  virtual int registerDevice(const bool &activate_on_registration);

  /**
   * Call the service to send reference commands to the device and wait for the response. Before sending the references,
   * correct their direction with the \p motor_axis_direction.
   * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
   * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
   * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
   * \return \p 0 on success.
   * \sa getMeasurements()
   */
  virtual int setCommands(const std::vector<double> &commands);

 private:
  /**
   * Add the namespace prefix stored in the private \p namespace_ to all the elements of the given vector.
   * \param vector Any vector of strings.
   * \return The namespaced vector.
   */
  std::vector<std::string> addNamespacePrefix(const std::vector<std::string> &vector);

  /**
   * Construct a \p qb_device_msgs::StateStamped message of the whole device state with the data retrieved during the
   * \p read() and publish it to a namespaced \p "~state" topic.
   * \sa read()
   */
  void publish();

  /**
   * Wait until the device is registered in the Communication Handler.
   * \sa deregisterDevice(), registerDevice()
   */
  void waitForRegistration();

  /**
   * Wait until all the services advertised by the Communication Handler are active.
   */
  void waitForServices();
};
typedef std::shared_ptr<qbDeviceHW> qbDeviceHWPtr;
}  // namespace qb_device_hardware_interface

#endif // QB_DEVICE_HARDWARE_INTERFACE_H