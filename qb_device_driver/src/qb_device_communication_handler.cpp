/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2021, qbrobotics®
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
  
}

qbDeviceCommunicationHandler::qbDeviceCommunicationHandler(qb_device_driver::qbDeviceAPIPtr device_api)
    : spinner_(11),  // 10 is the maximum number of connected serial ports (cr. API)
      node_handle_(ros::NodeHandle()),
      activate_motors_(node_handle_.advertiseService("/communication_handler/activate_motors", &qbDeviceCommunicationHandler::activateCallback, this)),
      deactivate_motors_(node_handle_.advertiseService("/communication_handler/deactivate_motors", &qbDeviceCommunicationHandler::deactivateCallback, this)),
      get_info_(node_handle_.advertiseService("/communication_handler/get_info", &qbDeviceCommunicationHandler::getInfoCallback, this)),
      get_measurements_(node_handle_.advertiseService("/communication_handler/get_measurements", &qbDeviceCommunicationHandler::getMeasurementsCallback, this)),
      initialize_device_(node_handle_.advertiseService("/communication_handler/initialize_device", &qbDeviceCommunicationHandler::initializeCallback, this)),
      set_commands_(node_handle_.advertiseService("/communication_handler/set_commands", &qbDeviceCommunicationHandler::setCommandsCallback, this)),
      set_pid_(node_handle_.advertiseService("/communication_handler/set_pid", &qbDeviceCommunicationHandler::setPIDCallback, this)),
      set_control_mode_(node_handle_.advertiseService("/communication_handler/set_control_mode", &qbDeviceCommunicationHandler::setControlModeCallback, this)),
      connection_state_publisher_(node_handle_.advertise<qb_device_msgs::ConnectionState>("/communication_handler/connection_state",1)),
      
      device_api_(device_api) {
  while (!getSerialPortsAndDevices(1)) {
    ROS_WARN_STREAM_NAMED("communication_handler", "[CommunicationHandler] is waiting for devices...");
    ros::Duration(1.0).sleep();
  }
  check_connection_status_timer_ = node_handle_.createWallTimer(ros::WallDuration(1), &qbDeviceCommunicationHandler::checkConnectionAndPublish, this);
  spinner_.start();
  
}

qbDeviceCommunicationHandler::~qbDeviceCommunicationHandler() {
  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] Attempt to terminate the process properly...");
  for (auto const &file_descriptor : file_descriptors_) {
    close(file_descriptor.first);
  }
}

int qbDeviceCommunicationHandler::activate(const int &id, const bool &command, const int &max_repeats) {
  std::string command_prefix = command ? "" : "de";
  bool status = false;
  int failures = 0;

  failures = isActive(id, max_repeats, status);
  if (status != command) {
    device_api_->activate(&file_descriptors_.at(connected_devices_.at(id)), id, command);
    failures = std::max(failures, isActive(id, max_repeats, status));
    if (status != command) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"communication_handler", "[CommunicationHandler] device [" << id << "] fails on " << command_prefix << "activation.");
      return -1;
    }
    ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] motors have been " << command_prefix << "activated!");
    return failures;
  }
  ROS_DEBUG_STREAM_NAMED("communication_handler", "[CommunicationHandler] device [" << id << "] motors were already " << command_prefix << "activated!");
  return failures;
}

int qbDeviceCommunicationHandler::activate(const int &id, const int &max_repeats) {
  return activate(id, true, max_repeats);
}

bool qbDeviceCommunicationHandler::activateCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.message = "Device [" + std::to_string(request.id) + "] is not connected.";
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.message = "Device [" + std::to_string(request.id) + "] activation.";
  response.failures = activate(request.id, request.max_repeats);
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::close(const std::string &serial_port) {
  if (!isInOpenMap(serial_port)) {
    ROS_WARN_STREAM_NAMED("communication_handler", "[CommunicationHandler] has not handled [" << serial_port << "].");
    return 0;  // no error: the communication is close anyway
  }

  for (auto const &device : connected_devices_) {
    if (device.second == serial_port) {
      deactivate(device.first, 3);
    }
  }
  connected_devices_.clear();
  ROS_INFO_STREAM_NAMED("communication_handler", "# of connected devices " << connected_devices_.size());
  device_api_->close(&file_descriptors_.at(serial_port));
  file_descriptors_.erase(serial_port);
  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] does not handle [" << serial_port << "] anymore.");
  return 0;
}

int qbDeviceCommunicationHandler::deactivate(const int &id, const int &max_repeats) {
  return activate(id, false, max_repeats);
}

bool qbDeviceCommunicationHandler::deactivateCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.message = "Device [" + std::to_string(request.id) + "] is not connected.";
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.message = "Device [" + std::to_string(request.id) + "] deactivation.";
  response.failures = deactivate(request.id, request.max_repeats);
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::getCurrents(const int &id, const int &max_repeats, std::vector<short int> &currents) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  currents.resize(2);  // required by 'getCurrents()'
  while (failures <= max_repeats) {
    if (device_api_->getCurrents(&file_descriptors_.at(connected_devices_.at(id)), id, currents) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::getInfo(const int &id, const int &max_repeats, std::string &info) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  while (failures <= max_repeats) {
    info = device_api_->getInfo(&file_descriptors_.at(connected_devices_.at(id)), id);
    if (info == "") {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::getInfoCallback(qb_device_srvs::TriggerRequest &request, qb_device_srvs::TriggerResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.message = "Device [" + std::to_string(request.id) + "] is not connected.";
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = getInfo(request.id, request.max_repeats, response.message);  // blocks while reading
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::getCommands(const int &id, const int &max_repeats, std::vector<short int> &commands) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  commands.resize(2);  // required by 'getCurrents()'
  while (failures <= max_repeats) {
    if (device_api_->getCommands(&file_descriptors_.at(connected_devices_.at(id)), id, commands) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::getMeasurements(const int &id, const int &max_repeats, std::vector<short int> &currents, std::vector<short int> &positions) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  currents.resize(2);
  positions.resize(3);
  std::vector<short int> measurements(5, 0);  // required by 'getMeasurements()'
  while (failures <= max_repeats) {
    if (device_api_->getMeasurements(&file_descriptors_.at(connected_devices_.at(id)), id, measurements) < 0) {
      failures++;
      continue;
    }
    std::copy(measurements.begin(), measurements.begin()+2, currents.begin());
    std::copy(measurements.begin()+2, measurements.end(), positions.begin());
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::getMeasurementsCallback(qb_device_srvs::GetMeasurementsRequest &request, qb_device_srvs::GetMeasurementsResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = 0;  // need to return true even if both 'get_currents' and 'get_positions' are set to false
  if (request.get_currents && request.get_positions) {
    if (!request.get_distinct_packages) {
      response.failures += getMeasurements(request.id, request.max_repeats, response.currents, response.positions);  // blocks while reading
    }
    else {
      response.failures += getCurrents(request.id, request.max_repeats, response.currents)
                          + getPositions(request.id, request.max_repeats, response.positions);  // both block while reading
    }
  }
  else if (request.get_currents) {
    response.failures += getCurrents(request.id, request.max_repeats, response.currents);  // blocks while reading
  }
  else if (request.get_positions) {
    response.failures += getPositions(request.id, request.max_repeats, response.positions);  // blocks while reading
  }

  if (request.get_commands) {
    response.failures += getCommands(request.id, request.max_repeats, response.commands);  // blocks while reading
  }

  response.stamp = ros::Time::now();
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions) {
  std::vector<int> input_mode = {-1};
  std::vector<int> control_mode = {-1};
  device_api_->getParameters(&file_descriptors_.at(connected_devices_.at(id)), id, input_mode, control_mode, resolutions, limits);
  if (!input_mode.front() && (!control_mode.front() || control_mode.front() == 4)) {  // both input and control modes equals 0 are required, i.e. respectively USB connected and position controlled (also deflection control is possible with qbmoves)
    return 0;
  }
  return -1;
}

int qbDeviceCommunicationHandler::getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  positions.resize(3);  // required by 'getPositions()'
  while (failures <= max_repeats) {
    if (device_api_->getPositions(&file_descriptors_.at(connected_devices_.at(id)), id, positions) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::getSerialPortsAndDevices(const int &max_repeats) {
  std::map<int, std::string> connected_devices;
  std::array<char[255], 10> serial_ports;
  int serial_ports_number = device_api_->getSerialPorts(serial_ports);
  for (int i=0; i<serial_ports_number; i++) {
    int failures = 0;
    while (failures <= max_repeats) {
      if (open(serial_ports.at(i)) != 0) {
        failures++;
        continue;
      }
      break;
    }
    if (failures >= max_repeats) {
      continue;
    }

    // 'serial_protectors_' is not cleared because of the previously acquired lock, do not do it!
    serial_protectors_.insert(std::make_pair(serial_ports.at(i), std::make_unique<std::mutex>()));  // never override

    int at_least_one_device_found = 0;
    std::array<char, 255> devices;
    int devices_number = device_api_->getDeviceIds(&file_descriptors_.at(serial_ports.at(i)), devices);
    for (int j=0; j<devices_number; j++) {
      if (devices.at(j) == 120 || devices.at(j) == 0) {
        continue;  // ID 120 is reserved for dummy board which should not be considered as a connected device (ID 0 is for sure an error)
      }
      // actually a std::map does not let same-id devices on distinct serial ports
      connected_devices.insert(std::make_pair(static_cast<int>(devices.at(j)), serial_ports.at(i)));
      at_least_one_device_found++;
    }

    if (!at_least_one_device_found) {
      ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] no qbrobotics devices found on [" << serial_ports.at(i) << "]");
      close(serial_ports.at(i));
    }
  }

  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] has found [" << connected_devices.size() << "] devices connected:");
  for (auto const &device : connected_devices) {
    ROS_INFO_STREAM_NAMED("communication_handler", "                       - device [" << device.first << "] connected through [" << device.second << "]");
  }

  connected_devices_ = connected_devices;
  return connected_devices_.size();
}

int qbDeviceCommunicationHandler::isActive(const int &id, const int &max_repeats, bool &status) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  status = false;
  while (failures <= max_repeats) {
    if (!device_api_->getStatus(&file_descriptors_.at(connected_devices_.at(id)), id, status)) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::isConnected(const int &id, const int &max_repeats) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  while (failures <= max_repeats) {
    if (!device_api_->getStatus(&file_descriptors_.at(connected_devices_.at(id)), id)) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::isInConnectedSet(const int &id) {
  return connected_devices_.count(id);
}

bool qbDeviceCommunicationHandler::isInOpenMap(const std::string &serial_port) {
  return file_descriptors_.count(serial_port);
}

int qbDeviceCommunicationHandler::open(const std::string &serial_port) {
  if (!std::regex_match(serial_port, std::regex("/dev/ttyUSB[[:digit:]]+"))) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"communication_handler", "[CommunicationHandler] fails while opening [" << serial_port << "] because it does not match the expected pattern [/dev/ttyUSB*].");
    return -1;
  }

  if (isInOpenMap(serial_port)) {
    ROS_DEBUG_STREAM_NAMED("communication_handler", "[CommunicationHandler] already handles [" << serial_port << "].");
    return 0;  // no error: the communication is open anyway
  }

  device_api_->open(&file_descriptors_[serial_port], serial_port);  // also create a pair in the map
  if(file_descriptors_.at(serial_port).file_handle == INVALID_HANDLE_VALUE) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(60 ,"communication_handler", "[CommunicationHandler] fails while opening [" << serial_port << "] and sets errno [" << strerror(errno) << "].");
    // remove file descriptor entry
    file_descriptors_.erase(serial_port);
    return -1;
  }

  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] handles [" << serial_port << "].");
  return 0;
}

bool qbDeviceCommunicationHandler::initializeCallback(qb_device_srvs::InitializeDeviceRequest &request, qb_device_srvs::InitializeDeviceResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  std::vector<std::unique_lock<std::mutex>> serial_locks;  // need to lock on all the serial resources to scan for new ports/devices
  for (auto const &mutex : serial_protectors_) {
    serial_locks.push_back(std::unique_lock<std::mutex>(*mutex.second));
  }

  if (request.rescan || !isInConnectedSet(request.id)) {
    // update connected devices
    ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] Rescanning...");
    check_connection_status_timer_.stop();
    getSerialPortsAndDevices(request.max_repeats);
    check_connection_status_timer_.start();
  }
  if (!isInConnectedSet(request.id) || !isReliable(isConnected(request.id, request.max_repeats), request.max_repeats)) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while initializing device [" << request.id << "] because it is not connected.");
    response.message = "Device [" + std::to_string(request.id) + "] initialization fails because it is not connected.";
    response.success = false;
    return true;
  }

  if (getParameters(request.id, response.info.position_limits, response.info.encoder_resolutions)) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while initializing device [" << request.id << "] because it requires 'USB' input mode and 'Position' control mode.");
    response.message = "Device [" + std::to_string(request.id) + "] initialization fails because it requires 'USB' input mode and 'Position' control mode.";
    response.success = false;
    return true;
  }
  if (request.activate) {
    response.failures = activate(request.id, request.max_repeats);
    response.success = isReliable(response.failures, request.max_repeats);
    if (!response.success) {
      ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] has not initialized device [" << request.id << "] because it cannot activate its motors (please, check the motor positions).");
      response.message = "Device [" + std::to_string(request.id) + "] initialization fails because it cannot activate the device (please, check the motor positions).";
      return true;
    }
  }
  response.info.id = request.id;
  response.info.serial_port = connected_devices_.at(request.id);
  ROS_INFO_STREAM_NAMED("communication_handler", "[CommunicationHandler] has initialized device [" << request.id << "].");
  response.message = "Device [" + std::to_string(request.id) + "] initialization succeeds.";
  response.success = true;
  return true;
}

void qbDeviceCommunicationHandler::checkConnectionAndPublish(const ros::WallTimerEvent &timer_event) {
  qb_device_msgs::DeviceConnectionInfo device_info_;
  bool status = false;
  int failures = 0;
  const int max_repeats = 3;
  connection_state_msg_.devices.clear();

  for(auto devices : connected_devices_) {
    std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(devices.first)));
    failures = isActive(devices.first, max_repeats, status);
    if (failures >= max_repeats) { // too much consecutive failues
      device_info_.is_active = false;
    } else {
      device_info_.is_active = status;
    }
    device_info_.id = devices.first;
    device_info_.port = devices.second;
    connection_state_msg_.devices.push_back(device_info_);
  }

  connection_state_publisher_.publish(connection_state_msg_);
}

int qbDeviceCommunicationHandler::setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  commands.resize(2);  // required by 'setCommandsAndWait()'
  while (failures <= max_repeats) {
    if (device_api_->setCommandsAndWait(&file_descriptors_.at(connected_devices_.at(id)), id, commands) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::setCommandsAsync(const int &id, std::vector<short int> &commands) {
  // qbhand sets only inputs.at(0), but setCommandsAsync expects two-element vector (ok for both qbhand and qbmove)
  commands.resize(2);  // required by 'setCommandsAsync()'
  device_api_->setCommandsAsync(&file_descriptors_.at(connected_devices_.at(id)), id, commands);
  return 0;  // note that this is a non reliable method
}

bool qbDeviceCommunicationHandler::setCommandsCallback(qb_device_srvs::SetCommandsRequest &request, qb_device_srvs::SetCommandsResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  if (request.set_commands) {
    if (!request.set_commands_async) {
      response.failures = setCommandsAndWait(request.id, request.max_repeats, request.commands);  // blocking function
    }
    else {
      response.failures = setCommandsAsync(request.id, request.commands);  // non-blocking (unreliable) function
    }
  }
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::setPID(const int &id, const int &max_repeats, std::vector<float> &pid) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  while (failures <= max_repeats) {
    if (device_api_->setPID(&file_descriptors_.at(connected_devices_.at(id)), id, pid) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::setPIDCallback(qb_device_srvs::SetPIDRequest &request, qb_device_srvs::SetPIDResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  if (request.p < 0 || request.p > 1 || request.i < 0 || request.i > 0.1 || request.d < 0 || request.d > 0.1) {  //TODO: set hardcoded properly
    ROS_ERROR_STREAM_NAMED("communication_handler","PID parameters are not in their acceptable ranges");
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  std::vector<float> pid({request.p, request.i, request.d});
  response.failures = setPID(request.id, request.max_repeats, pid);  // blocking function
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::setControlMode(const int &id, const int &max_repeats, uint8_t &control_id) {
  int failures = 0;
  while (failures <= max_repeats) {
    if (device_api_->setControlMode(&file_descriptors_.at(connected_devices_.at(id)), id, control_id) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}


bool qbDeviceCommunicationHandler::setControlModeCallback(qb_device_srvs::SetControlModeRequest &request, qb_device_srvs::SetControlModeResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  uint8_t control_id;
  if (request.control == "position") {
    control_id = 0;
  } else if (request.control == "deflection") {
    control_id = 4;
  } else {
    ROS_ERROR_STREAM_NAMED("communication_handler","Not valid control mode request.");
    response.success = false;
    return true;
  }
  response.failures = setControlMode(request.id, request.max_repeats, control_id);  // blocking function
  response.success = isReliable(response.failures, request.max_repeats);
  return true; 
}