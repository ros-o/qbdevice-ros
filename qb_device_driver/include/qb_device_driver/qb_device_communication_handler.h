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

#ifndef QB_DEVICE_COMMUNICATION_HANDLER_H
#define QB_DEVICE_COMMUNICATION_HANDLER_H

// Standard libraries
#include <regex>
#include <unordered_set>

// ROS libraries
#include <ros/ros.h>

// internal libraries
#include <qb_device_driver/qb_device_driver.h>
#include <qb_device_srvs/qb_device_srvs.h>

namespace qb_device_communication_handler {
/**
 * The Communication Handler class is aimed to instantiate a ROS node which provides several ROS services to
 * communicate with one - or many - qbrobotics devices connected to the ROS ecosystem.
 *
 * Each node which manage a single qbrobotics device must register its device ID in the Communication Handler and
 * interact with it through specific ROS services - blocking by nature. The Communication Handler essentially manage
 * the shared serial port resources and lets all the registered nodes to get access to them. This could seem a
 * bottleneck in the architecture, but the real bottleneck is the shared resource itself and it is also necessary
 * since the multi-process nature of ROS does not easily allow to share common resources among distinct processes.
 * Actually the implementation of qbrobotics classes could be reshaped to exploit the multi-threading approach, e.g.
 * using [nodelet](http://wiki.ros.org/nodelet) or similar plugins each device node could handle the communication
 * process by itself, without the intermediary Communication Handler. The advantage of such a restyle in term of
 * communication speed does not worth the effort though.
 */
class qbDeviceCommunicationHandler {
 public:
  /**
   * Wait until at least one device is connected and then initialize the Communication Handler.
   * \sa getSerialPortsAndDevices()
   */
  qbDeviceCommunicationHandler();

  /**
   * Allow to mock the API for unit testing. It is called from the default constructor with the real API smart pointer,
   * but it can be called with a pointer to the mock class inherited from the \p qbDeviceAPI itself.
   * \param device_api The shared pointer to the current API derived from \p qb_device_driver::qbDeviceAPI.
   */
  qbDeviceCommunicationHandler(qb_device_driver::qbDeviceAPIPtr device_api);

  /**
   * Close all the still open serial ports.
   * \sa close()
   */
  virtual ~qbDeviceCommunicationHandler();

 protected:
  /**
   * Activate the motors of the given device. Do nothing if the device is not connected or registered in the
   * Communication Handler.
   * \param id The ID of the device to be activated, in range [\p 1, \p 128].
   * \sa activate(const int &, const bool &), isActive()
   */
  virtual int activate(const int &id);

  /**
   * Activate the motors of the device relative to the node requesting the service.
   * \param request Contains only the device ID of the node which is requesting the service.
   * \param response Provides a \p success flag and a \p message string to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa activate(const int &)
   */
  bool activateCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response);

  /**
   * Close the communication with all the devices connected to the given serial port.
   * \param serial_port The serial port which has to be closed, e.g. \p /dev/ttyUSB0.
   * \sa qb_device_driver::qbDeviceAPI::close(), isOpen(), open()
   */
  virtual int close(const std::string &serial_port);

  /**
   * Deactivate the motors of the given device. Do nothing if the device is not connected or registered in the
   * Communication Handler.
   * \param id The ID of the device to be deactivated, in range [\p 1, \p 128].
   * \sa activate(const int &, const bool &), isActive()
   */
  virtual int deactivate(const int &id);

  /**
   * Deactivate the motors of the device relative to the node requesting the service.
   * \param request Contains only the device ID of the node which is requesting the service.
   * \param response Provides a \p success flag and a \p message string to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa deactivate()
   */
  bool deactivateCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response);

  /**
   * Deregister the device relative to the node requesting the service from the Communication Handler. It will not be
   * able to request anymore services until a new registration is performed.
   * \param request Contains only the device ID of the node which is requesting the service.
   * \param response Provides a \p success flag and a \p message string to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa isRegistered(), registerCallback()
   */
  bool deregisterCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response);

  /**
   * Retrieve the printable configuration setup of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \return The configuration setup formatted as a plain text string (empty string on communication error).
   * \sa qb_device_driver::qbDeviceAPI::getInfo(), getParameters()
   */
  virtual std::string getInfo(const int &id);

  /**
   * Retrieve the printable configuration setup of the device relative to the node requesting the service.
   * \param request Contains only the device ID of the node which is requesting the service.
   * \param response Provides a \p success flag and a \p message string to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa getInfo()
   */
  bool getInfoCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response);

  /**
   * Retrieve the motor positions and currents of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
   * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
   * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
   * not directly actuated).
   * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
   * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
   * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
   * \return \p 0 on success.
   * \sa qb_device_driver::qbDeviceAPI::getCurrents(), qb_device_driver::qbDeviceAPI::getPositions(), setCommands()
   */
  virtual int getMeasurements(const int &id, std::vector<short int> &positions, std::vector<short int> &currents);

  /**
   * Retrieve the motor positions and currents of the device relative to the node requesting the service.
   * \param request Contains only the device ID of the node which is requesting the service.
   * \param response Provides a \p success flag and the two data vectors \p positions and \p currents to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa getMeasurements()
   */
  bool getMeasurementsCallback(qb_device_srvs::GetMeasurementsRequest &request, qb_device_srvs::GetMeasurementsResponse &response);

  /**
   * Retrieve some of the parameters from the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] limits The vector of motor position limits expressed in \em ticks: two values for each motor,
   * respectively [\p lower_limit, \p upper_limit].
   * \param[out] resolutions The vector of encoder resolutions, each in range [\p 0, \p 8]: one value for each encoder
   * (\b note: the \em qbmove has also the shaft encoder even if it is not actuated). The word "resolution" could be
   * misunderstood: taken the resolution \p r, \f$2^r\f$ is the number of turns of the wire inside the device mechanics.
   * It is used essentially to convert the measured position of the motors in \em ticks to \em radians or \em degrees.
   * \return \p 0 on success.
   * \sa qb_device_driver::qbDeviceAPI::getParameters(), getInfo()
   */
  virtual int getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions);

  /**
   * Scan for all the serial ports of type \p /dev/ttyUSB* detected in the system and retrieve all the qbrobotics
   * devices connected to them. For each device, store its ID in the private map \p connected_devices_, i.e. insert a
   * pair [\p device_id, \p serial_port]. The map \p connected_devices_ is constructed from scratch at each call.
   * \return \p 0 if at least one device is retrieved.
   * \sa qb_device_driver::qbDeviceAPI::getDeviceIds(), qb_device_driver::qbDeviceAPI::getSerialPorts(), isConnected()
   */
  virtual int getSerialPortsAndDevices();

  /**
   * Check whether the motors of the device specified by the given ID are active.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \return \p true if the device motors are on.
   * \sa qb_device_driver::qbDeviceAPI::getStatus()
   */
  virtual bool isActive(const int &id);

  /**
   * Check whether the physical device specified by the given ID is connected to the Communication Handler.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \return \p true if the given device belongs to the connected device vector, i.e. \p connected_devices_.
   * \sa getSerialPortsAndDevices()
   */
  virtual bool isConnected(const int &id);

  /**
   * Check whether the given serial port is managed by the Communication Handler, i.e. is open.
   * \param serial_port The name of the serial port of interest, e.g. \p /dev/ttyUSB0.
   * \return \p true if the given serial port belongs to the open file descriptor map, i.e. \p file_descriptors_.
   * \sa open()
   */
  virtual bool isOpen(const std::string &serial_port);

  /**
   * Check whether the device ROS node specified by the given ID is registered to the Communication Handler.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \return \p true if the given device belongs to the registered device vector, i.e. \p registered_devices_.
   * \sa deregisterCallback(), registerCallback()
   */
  virtual bool isRegistered(const int &id);

  /**
   * Register the device node requesting the service to the Communication Handler (i.e. it will be able to request
   * services provided by this node) if the relative physical device is connected through any serial port to the system
   * (if not found at first, also scan once for new devices and update the list of connected ones). If requested,
   * activate the device motors.
   * \param request Contains the device ID of the node which is requesting the service and a flag \p activate to turn
   * on the motors automatically during the registration process.
   * \param response Provides a \p success flag, a \p message string, and some additional device info to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa deregisterCallback(), getSerialPortsAndDevices(), isRegistered()
   */
  bool registerCallback(qb_device_srvs::RegisterDeviceRequest &request, qb_device_srvs::RegisterDeviceResponse &response);

  /**
   * Open the serial communication on the given serial port. On success, store the opened file descriptor in the
   * private map \p file_descriptors_, i.e. insert a pair [\p serial_port, \p file_descriptor].
   * \param serial_port The serial port which has to be opened, e.g. \p /dev/ttyUSB0.
   * \return \p 0 on success.
   * \sa qb_device_driver::qbDeviceAPI::open(), close(), isOpen()
   */
  virtual int open(const std::string &serial_port);

  /**
   * Send the reference command to the motors of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
   * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
   * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
   * \return \p 0 on success.
   * \sa qb_device_driver::qbDeviceAPI::setCommands(), getMeasurements()
   */
  virtual int setCommands(const int &id, std::vector<short int> &commands);

  /**
   * Send the reference command to the motors of the device relative to the node requesting the service.
   * \param request Contains the device ID and the reference \p commands of the node which is requesting the service.
   * \param response Provides a \p success flag to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa setCommands()
   */
  bool setCommandsCallback(qb_device_srvs::SetCommandsRequest &request, qb_device_srvs::SetCommandsResponse &response);

  /**
   * Check if all the already registered devices are completely initialized and ready for the control application. When
   * a device node requests this service, it is automatically considered ready; if and only if the \p ready_devices_
   * map equals the \p registered_devices_ one (i.e. all the nodes have called this service at least once), all the
   * control loops can start in a barely synchronized way.
   * \param request Contains only the device ID of the node which is requesting the service.
   * \param response Provides a \p success flag and a \p message string to the caller.
   * \return \p true on success (same of \p response.success).
   * \sa setCommands()
   * \todo Add synchronization in-the-loop (not only at startup).
   */
  bool syncNodesCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response);

 private:
  ros::AsyncSpinner spinner_;
  ros::NodeHandle node_handle_;
  //TODO: pack all the service in a std::map services_(name, server)
  ros::ServiceServer activate_motors_;
  ros::ServiceServer deactivate_motors_;
  ros::ServiceServer deregister_device_;
  ros::ServiceServer get_info_;
  ros::ServiceServer get_measurements_;
  ros::ServiceServer register_device_;
  ros::ServiceServer set_commands_;
  ros::ServiceServer sync_nodes_;
  qb_device_driver::qbDeviceAPIPtr device_api_;

  std::map<std::string, comm_settings> file_descriptors_;
  std::map<int, std::string> connected_devices_;
  std::unordered_set<int> registered_devices_;
  std::unordered_set<int> ready_devices_;

  /**
   * Activate (or deactivate, according to the given command) the motors of the given device. Do nothing if the device
   * is not connected or registered in the Communication Handler.
   * \param id The ID of the device to be activated (or deactivated), in range [\p 1, \p 128].
   * \param command \p true to turn motors on, \p false to turn them off.
   * \sa qb_device_driver::qbDeviceAPI::activate(), activate(const int &), deactivate(), isActive()
   */
  virtual int activate(const int &id, const bool &command);
};
}  // namespace qb_device_communication_handler

#endif // QB_DEVICE_COMMUNICATION_HANDLER_H