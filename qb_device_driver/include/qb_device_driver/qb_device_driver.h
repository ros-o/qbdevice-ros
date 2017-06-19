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

#ifndef QB_DEVICE_DRIVER_H
#define QB_DEVICE_DRIVER_H

// internal libraries
#include <qbmove_communications.h>

namespace qb_device_driver {
/**
 * This class wraps the qbrobotics device-independent API to easily use it within the Communication Handler ROS node.
 */
class qbDeviceAPI {
 public:
  /**
   * Activate (or deactivate, according to the given command) the motors of the device connected to the serial port
   * relative to the given file descriptor. Do nothing if the file descriptor is wrong or not open.
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device to be activated (or deactivated), in range [\p 1, \p 128].
   * \param activate_command \p true to turn motors on, \p false to turn them off.
   * \sa getStatus()
   */
  virtual void activate(comm_settings *file_descriptor, const int &id, bool activate_command) {
    commActivate(file_descriptor, id, static_cast<char>(activate_command));
  }

  /**
   * Close the communication with all the devices connected to the serial port relative to the given file descriptor.
   * Do nothing if the file descriptor is wrong or not open.
   * \param file_descriptor The file descriptor of the serial port that has to be closed.
   * \sa open()
   */
  virtual void close(comm_settings *file_descriptor) {
    closeRS485(file_descriptor);
  }

  /**
   * Retrieve the motor currents of the given device.
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
   * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
   * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
   * \return \p 0 on success.
   * \sa getPositions(), setCommands()
   */
  virtual int getCurrents(comm_settings *file_descriptor, const int &id, std::vector<short int> &currents) {
    currents.resize(2);
    return commGetCurrents(file_descriptor, id, currents.data());
  }

  /**
   * Retrieve the list of devices connected to the serial port of the given file descriptor.
   * \param file_descriptor The file descriptor of the serial port to be scanned.
   * \param device_ids The retrieved device ID vector, each in range [\p 1, \p 128].
   * \return The number of devices connected to the given serial port.
   * \sa getSerialPorts()
   */
  virtual int getDeviceIds(comm_settings *file_descriptor, std::array<char, 255> &device_ids) {
    return RS485ListDevices(file_descriptor, device_ids.data());
  }

  /**
   * Retrieve the printable configuration setup of the given device.
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \return The configuration setup formatted as a plain text string (empty string on communication error).
   * \sa getParameters()
   */
  virtual std::string getInfo(comm_settings *file_descriptor, const int &id) {
    char info[2000];
    commGetInfo(file_descriptor, id, INFO_ALL, info);  // actually only INFO_ALL is supported
    return info;
  }

  /**
   * Retrieve some of the parameters from the given device.
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] limits The vector of motor position limits expressed in \em ticks: two values for each motor,
   * respectively [\p lower_limit, \p upper_limit].
   * \param[out] resolutions The vector of encoder resolutions, each in range [\p 0, \p 8]: one value for each encoder
   * (\b note: the \em qbmove has also the shaft encoder even if it is not actuated). The word "resolution" could be
   * misunderstood: taken the resolution \p r, \f$2^r\f$ is the number of turns of the wire inside the device mechanics.
   * It is used essentially to convert the measured position of the motors in \em ticks to \em radians or \em degrees.
   * \sa getParameter(), getInfo()
   */
  virtual void getParameters(comm_settings *file_descriptor, const int &id, std::vector<int> &limits, std::vector<int> &resolutions) {
    unsigned char parameter_buffer[2000];
    commGetParamList(file_descriptor, id, 0, NULL, 0, 0, parameter_buffer);
    ros::Duration(0.001).sleep();  // unexpected behaviour with no sleep

    getParameter<int32_t>(10, parameter_buffer, limits);  // hardcoded position limits
    getParameter<uint8_t>(6, parameter_buffer, resolutions);  // hardcoded encoder resolutions
  }

  /**
   * Retrieve the motor positions of the given device.
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
   * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
   * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
   * not directly actuated).
   * \return The number of positions retrieved, i.e. the number of encoder equipped on the given device.
   * \sa getCurrents(), setCommands()
   */
  virtual int getPositions(comm_settings *file_descriptor, const int &id, std::vector<short int> &positions) {
    positions.resize(3);
    return commGetMeasurements(file_descriptor, id, positions.data());
  }

  /**
   * Retrieve the list of serial ports connected to the system. Each port of the type \p /dev/ttyUSB* connected
   * to the system is considered a candidate to be scanned for qbrobotics devices.
   * \param serial_ports The array of serial ports connected to the system.
   * \return The number of serial ports connected to the system.
   * \sa getDeviceIds()
   */
  virtual int getSerialPorts(std::array<char[255], 10> &serial_ports) {
    return RS485listPorts(serial_ports.data());
  }

  /**
   * Retrieve the motor activation status of the given device.
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] activate_status \p true if motors are on, \p false if off.
   * \return \p 0 on success.
   * \sa activate()
   */
  virtual int getStatus(comm_settings *file_descriptor, const int &id, bool &activate_status) {
    char status;
    int result = commGetActivate(file_descriptor, id, &status);
    activate_status = status != 0;
    return result;
  }

  /**
   * Open the serial communication on the given serial port.
   * \param[out] file_descriptor The structure where the file descriptor has to be stored (filled with \p
   * INVALID_HANDLE_VALUE on error).
   * \param serial_port The serial port which has to be opened, e.g. \p /dev/ttyUSB0.
   * \sa close()
   */
  virtual void open(comm_settings *file_descriptor, const std::string &serial_port) {
    openRS485(file_descriptor, serial_port.c_str());
  }

  /**
   * Send the reference command to the motors of the given device.
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
   * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
   * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
   * \sa getCurrents(), getPositions()
   */
  virtual void setCommands(comm_settings *file_descriptor, const int &id, std::vector<short int> &commands) {
    ROS_ASSERT(commands.size() == 2);
    commSetInputs(file_descriptor, id, commands.data());
  }

 private:
  /**
   * Extract the specified parameter from the given buffer where all the device parameters are stored.
   * \tparam T The data type of the single field of the parameter to be retrieved, e.g. \p int32_t.
   * \param parameter_id The specific value of the parameter to be retrieved, mapped in the device firmware.
   * \param[out] parameter_vector The vector where the values are stored (\b note: it is initially cleared).
   * \sa getParameters()
   */
  template<class T>
  void getParameter(const int &parameter_id, unsigned char *parameter_buffer, std::vector<int> &parameter_vector) {
    parameter_vector.clear();
    int number_of_values = parameter_buffer[parameter_id*PARAM_BYTE_SLOT + 7];
    int value_size = sizeof(T);
    for (int i=0; i<number_of_values; i++) {
      T parameter_field = 0;
      for (int j=0; j<value_size; j++) {
        parameter_field += parameter_buffer[parameter_id*PARAM_BYTE_SLOT + 8 + i*value_size + value_size - j - 1] << (8 * j);
      }
      parameter_vector.push_back(parameter_field);
    }
  }
};
typedef std::shared_ptr<qbDeviceAPI> qbDeviceAPIPtr;
}  // namespace qb_device_driver

#endif // QB_DEVICE_DRIVER_H