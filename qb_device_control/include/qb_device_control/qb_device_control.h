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

#ifndef QB_DEVICE_CONTROL_H
#define QB_DEVICE_CONTROL_H

// Standard libraries
#include <regex>

// ROS libraries
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_manager/controller_manager.h>

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>

namespace qb_device_control {
/**
 * The qbrobotics Control interface provides all the common structures to control both the \em qbhand and \em qbmove
 * devices. It exploits the \p qb_device_hardware_interface::qbDeviceHW for serial communication with the hardware, but
 * it needs to be initialized with the proper derived HW interface, e.g. \p qbHandHW for the \em qbhand. The core part
 * is given by the \p update() which is called each time the timer triggers (the control loop frequency is settable
 * from the Parameter Server), and which implements the general control law: sensors reading from the device, control
 * action computation according to the loaded controller, and actuator reference commands writing to the hardware. This
 * class also provides a simple waypoints trajectory control which is automatically used in case waypoints are retrieved
 * from the Parameter Server.
 * \sa qb_hand_control::qbHandControl, qb_move_control::qbMoveControl
 */
class qbDeviceControl {
 public:
  /**
   * Initialize the control structures and wait until all the required ROS interfaces are started. In detail, initialize
   * the Control Manager with the given Device HardWare interface, i.e. the \p qbHandHW or \p qbMoveHW according to the
   * current qbrobotics device in use. Then wait until the Control Manager is started, the specific device controllers
   * are spawned and the Action Server is advertising the proper control action. Also retrieve a set of control
   * reference waypoints to perform a cyclic demo around them, automatically. If the set is not specified in the
   * Parameter Server (under the private \p "~waypoints" ROS param), wait until an external reference is given, e.g.
   * through the joint trajectory GUI or from a specific derived control class.
   * \param device The shared pointer to the current HW interface derived from \p qb_device_hardware_interface::qbDeviceHW.
   * \sa parseWaypoints(), waitForActionServer(), waitForControllerManager(), waitForControllers()
   */
  qbDeviceControl(qb_device_hardware_interface::qbDeviceHWPtr device);

  /**
   * Do nothing.
   */
  virtual ~qbDeviceControl();

  /**
   * Build a \p control_msgs::FollowJointTrajectoryAction goal with the given joint trajectory and make a call to the
   * Action Server (using the just created goal).
   * \param joint_trajectory The waypoint trajectory properly filled for all the joints.
   * \sa move(const std::map<double, std::vector<double>> &), setTrajectory()
   */
  void move(const trajectory_msgs::JointTrajectory &joint_trajectory);

  /**
   * A simple overload of the \p move(const trajectory_msgs::JointTrajectory &) to use directly the waypoint trajectory
   * map.
   * \param waypoints_map The waypoint trajectory map containing pairs of [\p time, \p joint_positions].
   * \sa move(const trajectory_msgs::JointTrajectory &), parseWaypoints(), setTrajectory()
   */
  void move(const std::map<double, std::vector<double>> &waypoints_map);

  /**
   * Retrieve the control waypoints from the Parameter Server. If not specified under the private \p "~waypoints" ROS
   * param, do nothing.
   * \param waypoint_map_name The name of the ROS parameter which contains the list (i.e. not mapping) of waypoints.
   * Each element is mapping which keys are [\p time, \p joint_positions] .
   * \param[out] waypoint_map The waypoint trajectory map containing pairs of [\p time, \p joint_positions]. If the \p
   * waypoint_map_name is not present in the Parameter Server the trajectory remains unchanged, otherwise it is cleared
   * and filled with the specified values.
   * \sa xmlCast()
   */
  void parseWaypoints(const std::string &waypoint_map_name, std::map<double, std::vector<double>> &waypoint_map);

  /**
   * Fill a \p trajectory_msgs::JointTrajectory ROS message with the given waypoint trajectory map containing pairs of
   * [\p time, \p joint_positions].
   * \param waypoints_map The waypoint trajectory map containing pairs of [\p time, \p joint_positions].
   * \return The filled \p trajectory_msgs::JointTrajectory ROS message.
   * \sa move(const trajectory_msgs::JointTrajectory &)
   * \todo Add support for \p joint_velocities and \p joint_accelerations as additional specification for waypoints.
   */
  trajectory_msgs::JointTrajectory setTrajectory(const std::map<double, std::vector<double>> &waypoints_map);

  /**
   * Wait until the action is completed or the given timeout is reached.
   * \param timeout The maximum amount of time to wait.
   * \return \p true if action has ended.
   */
  bool waitForResult(const ros::Duration &timeout);

 protected:
  ros::AsyncSpinner spinner_;
  ros::NodeHandle node_handle_;
  ros::ServiceClient sync_nodes_;
  ros::WallTimer control_timer_;
  ros::WallDuration control_duration_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
  controller_manager::ControllerManager controller_manager_;
  qb_device_hardware_interface::qbDeviceHWPtr device_;
  std::map<double, std::vector<double>> waypoint_trajectory_map_;

 private:
  /**
   * Do nothing.
   * \todo Add synchronization among nodes.
   */
  void actionActiveCallback();

  /**
   * Restart the waypoint trajectory automatically if waypoints have been retrieved in the initialization. Otherwise
   * do nothing.
   * \param state The final state of the action.
   * \param result The error code and error message (\p 0 if succeeds).
   * \sa move(const std::map<double, std::vector<double>> &)
   */
  void actionDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result);

  /**
   * Do nothing.
   * \param feedback The action feedback state.
   */
  void actionFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback);

  /**
   * Call the \p update() each time the timer triggers.
   * \param timer_event The timer event struct which stores timing info.
   * \sa update()
   */
  void controlCallback(const ros::WallTimerEvent &timer_event);

  /**
   * Read the current state from the HW, update all active controllers, and send the new references to the HW. The
   * control references follow the current implementation of the controllers in use.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   */
  void update(const ros::WallTime& time, const ros::WallDuration& period);

  /**
   * Wait until the specific Action Server is up. It is configured to search for an Action Server which advertise an
   * action ending with \p "_trajectory_controller/follow_joint_trajectory".
   * \sa qbDeviceControl()
   */
  void waitForActionServer();

  /**
   * Wait until the Controller Manager for the specific HardWare interface is up.
   * \sa qbDeviceControl()
   */
  void waitForControllerManager();

  /**
   * Wait until the namespaced controllers in the Parameter Server are loaded in the Controller Manager. Search for all
   * the parameters nested in the device namespace which end with \p "_controller" and which have a subparameter called
   * \p "type".
   * \sa qbDeviceControl()
   */
  void waitForControllers();

  /**
   * Wait until all the other device nodes registered on the Communication Handler are ready. This is a sort of
   * synchronization between nodes: the maximum delay is 1 millisecond.
   * \sa qbDeviceControl()
   */
  void waitForOtherNodes();

  /**
   * Cast an \p XmlRpcValue from \p TypeDouble, \p TypeInt or \p TypeBoolean to the specified template type. This is
   * necessary since XmlRpc does not handle conversion among basic types: it throws an exception if an improper cast is
   * invoked though (e.g. from int to double).
   * \tparam T The type to cast to.
   * \param xml_value The wrong-casted value.
   * \return The casted value.
   * \sa parseWaypoints()
   */
  template<class T> T xmlCast(XmlRpc::XmlRpcValue xml_value);
};
}  // namespace qb_device_control

#endif // QB_DEVICE_CONTROL_H