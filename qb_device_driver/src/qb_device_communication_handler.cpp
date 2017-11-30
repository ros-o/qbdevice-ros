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

#include <qb_device_driver/qb_device_communication_handler.h>

using namespace qb_device_communication_handler;

qbDeviceCommunicationHandler::qbDeviceCommunicationHandler()
    : qbDeviceCommunicationHandler(std::make_shared<qb_device_driver::qbDeviceAPI>()) {
  while (getSerialPortsAndDevices() != 0) {
    ROS_WARN_STREAM_NAMED("communication_handler", "[CommunicationHandler] is waiting for devices...");
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] has found [" << connected_devices_.size() << "] devices connected:");
  for (auto const device : connected_devices_) {
    ROS_INFO_STREAM_NAMED("communication_handler", "                       - device [" << device.first << "] connected through [" << device.second << "]");
  }

  spinner_.start();
}

qbDeviceCommunicationHandler::qbDeviceCommunicationHandler(qb_device_driver::qbDeviceAPIPtr device_api)
    : spinner_(1),
      node_handle_(ros::NodeHandle()),
      activate_motors_(node_handle_.advertiseService("/communication_handler/activate_motors", &qbDeviceCommunicationHandler::activateCallback, this)),
      deactivate_motors_(node_handle_.advertiseService("/communication_handler/deactivate_motors", &qbDeviceCommunicationHandler::deactivateCallback, this)),
      deregister_device_(node_handle_.advertiseService("/communication_handler/deregister_device", &qbDeviceCommunicationHandler::deregisterCallback, this)),
      get_info_(node_handle_.advertiseService("/communication_handler/get_info", &qbDeviceCommunicationHandler::getInfoCallback, this)),
      get_measurements_(node_handle_.advertiseService("/communication_handler/get_measurements", &qbDeviceCommunicationHandler::getMeasurementsCallback, this)),
      register_device_(node_handle_.advertiseService("/communication_handler/register_device", &qbDeviceCommunicationHandler::registerCallback, this)),
      set_commands_(node_handle_.advertiseService("/communication_handler/set_commands", &qbDeviceCommunicationHandler::setCommandsCallback, this)),
      sync_nodes_(node_handle_.advertiseService("/communication_handler/sync_nodes", &qbDeviceCommunicationHandler::syncNodesCallback, this)),
      device_api_(device_api) {
}

qbDeviceCommunicationHandler::~qbDeviceCommunicationHandler() {
  for (auto const file_descriptor : file_descriptors_) {
    close(file_descriptor.first);
  }
}

int qbDeviceCommunicationHandler::activate(const int &id, const bool &command) {
  std::string command_prefix = command ? "" : "de";
  if (!isRegistered(id) || !isConnected(id) || !isOpen(connected_devices_.at(id))) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] cannot " << command_prefix << "activate device [" << id << "] because it is not registered or even connected.");
    return -1;
  }

  if (isActive(id) != command) {
    device_api_->activate(&file_descriptors_.at(connected_devices_.at(id)), id, command);
    ros::Duration(0.001).sleep();
    if (isActive(id) != command) {
      ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] fails on " << command_prefix << "activation.");
      return -1;
    }
  }
  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] motors have been " << command_prefix << "activated!");
  return 0;
}

int qbDeviceCommunicationHandler::activate(const int &id) {
  return activate(id, true);
}

bool qbDeviceCommunicationHandler::activateCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  response.message = "Device [" + std::to_string(request.id) + "] activation.";
  response.success = !activate(request.id);
  return response.success;
}

int qbDeviceCommunicationHandler::close(const std::string &serial_port) {
  if (!isOpen(serial_port)) {
    ROS_WARN_STREAM_NAMED("communication_handler", "[CommunicationHandler] has not handled [" << serial_port << "].");
    return 0;  // no error: the communication is close anyway
  }

  for (auto const device : connected_devices_) {
    if (device.second == serial_port) {
      deactivate(device.first);
      connected_devices_.erase(device.first);
      registered_devices_.erase(device.first);
      ready_devices_.erase(device.first);
    }
  }
  device_api_->close(&file_descriptors_.at(serial_port));
  file_descriptors_.erase(serial_port);
  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] does not handle [" << serial_port << "] anymore.");
  return 0;
}

int qbDeviceCommunicationHandler::deactivate(const int &id) {
  return activate(id, false);
}

bool qbDeviceCommunicationHandler::deactivateCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  response.message = "Device [" + std::to_string(request.id) + "] deactivation.";
  response.success = !deactivate(request.id);
  return response.success;
}

bool qbDeviceCommunicationHandler::deregisterCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  // do nothing if inactive or not registered
  deactivate(request.id);
  registered_devices_.erase(request.id);
  ready_devices_.erase(request.id);

  response.message = "Device [" + std::to_string(request.id) + "] deregistration.";
  response.success = true;
  return response.success;
}

std::string qbDeviceCommunicationHandler::getInfo(const int &id) {
  if (!isRegistered(id) || !isConnected(id) || !isOpen(connected_devices_.at(id))) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while getting info of device [" << id << "] because it is not registered or even connected.");
    return "";
  }
  return device_api_->getInfo(&file_descriptors_.at(connected_devices_.at(id)), id);
}

bool qbDeviceCommunicationHandler::getInfoCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  // note: getInfo is a blocking function
  response.message = getInfo(request.id);
  response.success = response.message != "";
  return response.success;
}

int qbDeviceCommunicationHandler::getMeasurements(const int &id, std::vector<short int> &positions, std::vector<short int> &currents) {
  if (!isRegistered(id) || !isConnected(id) || !isOpen(connected_devices_.at(id))) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while getting measurements because device [" << id << "] is not registered or even connected.");
    return -1;
  }

  //FIXME: cannot use the 'commGetCurrAndMeas' API function because the command 'CMD_GET_CURR_AND_MEAS' is not
  //FIXME  implemented in the qbhand firmware... (it has been put only inside the qbmove firmware)
  // the API methods are called at most (i.e. very unlikely) three times to guarantee the correct identification of a real fault in the communication
  if (device_api_->getPositions(&file_descriptors_.at(connected_devices_.at(id)), id, positions) < 0 &&
      device_api_->getPositions(&file_descriptors_.at(connected_devices_.at(id)), id, positions) < 0 &&
      device_api_->getPositions(&file_descriptors_.at(connected_devices_.at(id)), id, positions) < 0) {
    return -1;
  }
  if (device_api_->getCurrents(&file_descriptors_.at(connected_devices_.at(id)), id, currents) < 0 &&
      device_api_->getCurrents(&file_descriptors_.at(connected_devices_.at(id)), id, currents) < 0 &&
      device_api_->getCurrents(&file_descriptors_.at(connected_devices_.at(id)), id, currents) < 0) {
    return -1;
  }
  return 0;
}

bool qbDeviceCommunicationHandler::getMeasurementsCallback(qb_device_srvs::GetMeasurementsRequest &request, qb_device_srvs::GetMeasurementsResponse &response) {
  response.positions.resize(3);
  response.currents.resize(2);
  ros::Duration(0.0001).sleep();  //TODO: fix API/firmware side (the sleep is mandatory with many devices)
  // note: getMeasurements is a blocking function
  response.success = !getMeasurements(request.id, response.positions, response.currents);
  return response.success;
}

int qbDeviceCommunicationHandler::getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions) {
  if (!isRegistered(id) || !isConnected(id) || !isOpen(connected_devices_.at(id))) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while getting parameters because device [" << id << "] is not registered or even connected.");
    return -1;
  }

  device_api_->getParameters(&file_descriptors_.at(connected_devices_.at(id)), id, limits, resolutions);
  return 0;
}

int qbDeviceCommunicationHandler::getSerialPortsAndDevices() {
  connected_devices_.clear();
  int result = -1;

  std::array<char[255], 10> serial_ports;
  for (int i=0; i<device_api_->getSerialPorts(serial_ports); i++) {
    if (open(serial_ports.at(i)) != 0) {
      continue;
    }

    std::array<char, 255> devices;
    for (int j=0; j<device_api_->getDeviceIds(&file_descriptors_.at(serial_ports.at(i)), devices); j++) {
      //FIXME: actually a std::map does not let same-id devices on distinct serial ports
      connected_devices_.insert(std::make_pair(static_cast<int>(devices.at(j)), serial_ports.at(i)));
      result = 0;
    }
  }

  // update registered devices if someone has been disconnected in the meanwhile
  for (auto const device_id : registered_devices_) {
    if (!isConnected(device_id)) {
      deactivate(device_id);
      registered_devices_.erase(device_id);
      ready_devices_.erase(device_id);
    }
  }

  return result;
}

bool qbDeviceCommunicationHandler::isActive(const int &id) {
  bool status = false;
  if (!isRegistered(id) || !isConnected(id) || !isOpen(connected_devices_.at(id))) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] is not registered or even connected.");
    return false;
  }
  // the API method is called at most (i.e. very unlikely) three times to guarantee the correct identification of a real fault in the communication
  if (device_api_->getStatus(&file_descriptors_.at(connected_devices_.at(id)), id, status) &&
      device_api_->getStatus(&file_descriptors_.at(connected_devices_.at(id)), id, status) &&
      device_api_->getStatus(&file_descriptors_.at(connected_devices_.at(id)), id, status)) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] is not even connected.");
    return false;
  }
  return status;
}

bool qbDeviceCommunicationHandler::isConnected(const int &id) {
  return connected_devices_.count(id);
}

bool qbDeviceCommunicationHandler::isOpen(const std::string &serial_port) {
  return file_descriptors_.count(serial_port);
}

bool qbDeviceCommunicationHandler::isRegistered(const int &id) {
  return registered_devices_.count(id);
}

int qbDeviceCommunicationHandler::open(const std::string &serial_port) {
  if (!std::regex_match(serial_port, std::regex("/dev/ttyUSB[[:digit:]]+"))) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while opening [" << serial_port << "] because it does not match the expected pattern [/dev/ttyUSB*].");
    return -1;
  }

  if (isOpen(serial_port)) {
    ROS_DEBUG_STREAM_NAMED("communication_handler", "[CommunicationHandler] already handles [" << serial_port << "].");
    return 0;  // no error: the communication is open anyway
  }

  device_api_->open(&file_descriptors_[serial_port], serial_port);  // also create a pair in the map
  if(file_descriptors_.at(serial_port).file_handle == INVALID_HANDLE_VALUE) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while opening [" << serial_port << "] and sets errno [" << strerror(errno) << "].");
    // remove file descriptor entry
    file_descriptors_.erase(serial_port);
    return -1;
  }

  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] handles [" << serial_port << "].");
  return 0;
}

bool qbDeviceCommunicationHandler::registerCallback(qb_device_srvs::RegisterDeviceRequest &request, qb_device_srvs::RegisterDeviceResponse &response) {
  if (!isConnected(request.id)) {
    // update connected devices
    getSerialPortsAndDevices();
    if (!isConnected(request.id)) {
      ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while registering device [" << request.id << "] because it is not connected.");
      response.message = "Device [" + std::to_string(request.id) + "] registration fails because it is not connected.";
      response.success = false;
      return response.success;
    }
  }

  response.success = true;
  registered_devices_.insert(request.id);
  if (request.activate) {
    response.success &= !activate(request.id);
  }
  response.success &= !getParameters(request.id, response.info.position_limits, response.info.encoder_resolutions);
  // actually the following are not used
  response.info.id = request.id;
  response.info.serial_port = connected_devices_.at(request.id);

  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] has registered device [" << request.id << "].");
  response.message = "Device [" + std::to_string(request.id) + "] registration succeeds.";
  return response.success;
}

int qbDeviceCommunicationHandler::setCommands(const int &id, std::vector<short int> &commands) {
  if(!isActive(id)) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while sending commands because device [" << id << "] is not active.");
    return -1;
  }

  // qbhand sets only inputs.at(0), but setCommands expects two-element vector
  commands.resize(2);
  // ok for both qbhand and qbmove
  device_api_->setCommands(&file_descriptors_.at(connected_devices_.at(id)), id, commands);
  return 0;
}

bool qbDeviceCommunicationHandler::setCommandsCallback(qb_device_srvs::SetCommandsRequest &request, qb_device_srvs::SetCommandsResponse &response) {
  // note: setCommands is a non-blocking function
  ros::Duration(0.0001).sleep();  //TODO: fix API/firmware side (the sleep is mandatory with many devices)
  response.success = !setCommands(request.id, request.commands);
  return response.success;
}

bool qbDeviceCommunicationHandler::syncNodesCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  response.success = false;
  if (isRegistered(request.id)) {
    ready_devices_.insert(request.id);
    response.message = "Device [" + std::to_string(request.id) + "] is ready.";
    response.success = ready_devices_.size() == registered_devices_.size();
  }
  return response.success;
}