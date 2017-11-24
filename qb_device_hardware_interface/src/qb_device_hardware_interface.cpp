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

#include <qb_device_hardware_interface/qb_device_hardware_interface.h>

using namespace qb_device_hardware_interface;

qbDeviceHW::qbDeviceHW(qb_device_transmission_interface::TransmissionPtr transmission, const std::vector<std::string> &actuators, const std::vector<std::string> &joints)
  : spinner_(1),
    node_handle_(ros::NodeHandle()),
    state_publisher_(node_handle_.advertise<qb_device_msgs::StateStamped>("state", 1)),
    activate_motors_(node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/activate_motors", true)),
    deactivate_motors_(node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/deactivate_motors", true)),
    deregister_device_(node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/deregister_device", true)),
    get_info_(node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/get_info", true)),
    get_measurements_(node_handle_.serviceClient<qb_device_srvs::GetMeasurements>("/communication_handler/get_measurements", true)),
    register_device_(node_handle_.serviceClient<qb_device_srvs::RegisterDevice>("/communication_handler/register_device", true)),
    set_commands_(node_handle_.serviceClient<qb_device_srvs::SetCommands>("/communication_handler/set_commands", true)),
    device_(ros::param::param<int>("~device_id", 1)),
    namespace_(ros::param::param<std::string>("~namespace", "qbdevice_" + std::to_string(device_.id))) {
  spinner_.start();
  //TODO: investigate namespace (ros::this_node::getNamespace() returns "//namespace" instead of "/namespace")
  initializeResources(actuators, joints);
  initializeInterfaces(transmission);

  waitForServices();
  waitForRegistration();
  ROS_INFO_STREAM(getInfo());
  //TODO: add a check on the device control mode (warn if it is not 'position')
}

qbDeviceHW::~qbDeviceHW() {
  deregisterDevice();
  spinner_.stop();
}

int qbDeviceHW::activateMotors() {
  if (activate_motors_) {
    qb_device_srvs::Trigger srv;
    srv.request.id = device_.id;
    activate_motors_.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot activate device [" << device_.id << "].");
      return -1;
    }
    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] device [" << device_.id << "] motors are active!");
    return 0;
  }
  ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] service [activate_motors] is no longer advertised.");
  return -1;
}

std::vector<std::string> qbDeviceHW::addNamespacePrefix(const std::vector<std::string> &vector) {
  std::vector<std::string> namespaced_vector;
  for (auto &elem : vector) {
    namespaced_vector.push_back(namespace_ + "_" + elem);
  }
  return namespaced_vector;
}

int qbDeviceHW::deactivateMotors() {
  if (deactivate_motors_) {
    qb_device_srvs::Trigger srv;
    srv.request.id = device_.id;
    deactivate_motors_.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot deactivate device [" << device_.id << "].");
      return -1;
    }
    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] device [" << device_.id << "] motors are inactive.");
    return 0;
  }
  ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] service [deactivate_motors] is no longer advertised.");
  return -1;
}

int qbDeviceHW::deregisterDevice() {
  if (deregister_device_) {
    qb_device_srvs::Trigger srv;
    srv.request.id = device_.id;
    deregister_device_.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot deregister device [" << device_.id << "] from [CommunicationHandler].");
      return -1;
    }
    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] device [" << device_.id << "] is no longer registered in [CommunicationHandler].");
    return 0;
  }
  ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] service [deregister_device] is no longer advertised.");
  return -1;
}

std::string qbDeviceHW::getInfo() {
  if (get_info_) {
    qb_device_srvs::Trigger srv;
    srv.request.id = device_.id;
    get_info_.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot get info from device [" << device_.id << "].");
      return "";
    }
    return srv.response.message;
  }
  ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] service [get_info] is no longer advertised.");
  return "";
}

int qbDeviceHW::getMeasurements(std::vector<double> &positions, std::vector<double> &currents) {
  if (get_measurements_) {
    qb_device_srvs::GetMeasurements srv;
    srv.request.id = device_.id;
    get_measurements_.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot get measurements from device [" << device_.id << "].");
      return -1;
    }

    // positions and currents cannot be resized because actuators state handle in the caller stores their addresses
    for (int i=0; i<positions.size() && i<srv.response.positions.size(); i++) {
      positions.at(i) = srv.response.positions.at(i) * device_.motor_axis_direction;
    }
    for (int i=0; i<currents.size() && i<srv.response.currents.size(); i++) {
      currents.at(i) = srv.response.currents.at(i);
    }
    return 0;
  }
  ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] service [get_measurements] is no longer advertised.");
  return -1;
}

void qbDeviceHW::initializeInterfaces(qb_device_transmission_interface::TransmissionPtr transmission) {
  interfaces_.initialize(this, joints_);
  joint_limits_.initialize(joints_, urdf_model_, interfaces_.joint_position);
  transmission_.initialize(ros::param::param<std::string>("~transmission", "transmission"), transmission, actuators_, joints_);
}

void qbDeviceHW::initializeResources(const std::vector<std::string> &actuators, const std::vector<std::string> &joints) {
  actuators_.setJoints(ros::param::param<std::vector<std::string>>("~actuators", addNamespacePrefix(actuators)));
  joints_.setJoints(ros::param::param<std::vector<std::string>>("~joints", addNamespacePrefix(joints)));

  if (!urdf_model_.initParam(ros::param::param<std::string>("~robot_description", "robot_description"))) {
    ROS_ERROR_STREAM("Device [" << device_.id << "] fails while retrieving the urdf model from the parameter server.");
  }
}

void qbDeviceHW::publish() {
  qb_device_msgs::StateStamped msg;

  msg.device_info.id = device_.id;
  msg.device_info.serial_port = device_.serial_port;
  msg.device_info.position_limits = device_.position_limits;
  msg.device_info.encoder_resolutions = device_.encoder_resolutions;

  for (int i=0; i<actuators_.names.size(); i++) {
    qb_device_msgs::ResourceData msg_actuator_data;
    msg_actuator_data.name = actuators_.names.at(i);
    msg_actuator_data.position = actuators_.positions.at(i);
    msg_actuator_data.velocity = actuators_.velocities.at(i);
    msg_actuator_data.effort = actuators_.efforts.at(i);
    msg_actuator_data.command = actuators_.commands.at(i);
    msg.state.actuators.push_back(msg_actuator_data);
  }

  for (int i=0; i<joints_.names.size(); i++) {
    qb_device_msgs::ResourceData msg_joint_data;
    msg_joint_data.name = joints_.names.at(i);
    msg_joint_data.position = joints_.positions.at(i);
    msg_joint_data.velocity = joints_.velocities.at(i);
    msg_joint_data.effort = joints_.efforts.at(i);
    msg_joint_data.command = joints_.commands.at(i);
    msg.state.joints.push_back(msg_joint_data);
  }

  msg.header.frame_id = namespace_;
  msg.header.stamp = ros::Time::now();

  state_publisher_.publish(msg);
}

void qbDeviceHW::read(const ros::Time& time, const ros::Duration& period) {
  // store old actuator positions
  std::vector<double> actuator_position_old(actuators_.positions);
  // read actuator state from the hardware
  getMeasurements(actuators_.positions, actuators_.efforts);
  // update velocities which are computed by differentiation, e.g. (pos_new - pos_old)/delta_t
  for (int i=0; i<actuators_.names.size(); i++) {
    actuators_.velocities.at(i) = (actuators_.positions.at(i) - actuator_position_old.at(i)) / period.toSec();
  }

  // no needs to enforce joint limits: they come from the real device

  // propagate current actuator state to joints
  transmission_.actuator_to_joint_state.propagate();

  // make data available for other ROS nodes
  publish();
}

int qbDeviceHW::registerDevice(const bool &activate_on_registration) {
  if (register_device_) {
    qb_device_srvs::RegisterDevice srv;
    srv.request.id = device_.id;
    srv.request.activate = activate_on_registration;
    register_device_.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot register device [" << device_.id << "] to [CommunicationHandler].");
      return -1;
    }
    device_.serial_port = srv.response.info.serial_port;
    device_.position_limits = srv.response.info.position_limits;
    device_.encoder_resolutions = srv.response.info.encoder_resolutions;
    ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] device [" << device_.id << "] is registered in [CommunicationHandler].");
    return 0;
  }
  ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] service [register_device] is no longer advertised.");
  return -1;
}

int qbDeviceHW::setCommands(const std::vector<double> &commands) {
  if (set_commands_) {
    qb_device_srvs::SetCommands srv;
    srv.request.id = device_.id;

    for (auto command : commands) {
      srv.request.commands.push_back(static_cast<short int>(command) * device_.motor_axis_direction);
    }

    set_commands_.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot send commands to device [" << device_.id << "].");
      return -1;
    }
    return 0;
  }
  ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] service [set_commands] is no longer advertised.");
  return -1;
}

void qbDeviceHW::waitForRegistration() {
  while(registerDevice(true)) ;
}

void qbDeviceHW::waitForServices() {
  //TODO: when packed use a for(auto ...)
  ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] is waiting for services to be advertise by [CommunicationHandler].");
  activate_motors_.waitForExistence();
  deactivate_motors_.waitForExistence();
  deregister_device_.waitForExistence();
  get_info_.waitForExistence();
  get_measurements_.waitForExistence();
  register_device_.waitForExistence();
  set_commands_.waitForExistence();
  ROS_INFO_STREAM_NAMED("device_hw", "[DeviceHW] is connected to all the services advertise by [CommunicationHandler].");
}

void qbDeviceHW::write(const ros::Time& time, const ros::Duration& period) {
  // enforce joint limits for all registered interfaces
  joint_limits_.enforceLimits(period);

  // propagate joint commands to actuators
  transmission_.joint_to_actuator_position.propagate();

  // send actuator commands to the hardware
  setCommands(actuators_.commands);
}