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

#include <qb_device_control/qb_device_control.h>

using namespace qb_device_control;

qbDeviceControl::qbDeviceControl(qb_device_hardware_interface::qbDeviceHWPtr device)
    : spinner_(1),
      device_(device),
      node_handle_(ros::NodeHandle()),
      sync_nodes_(node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/sync_nodes", true)),
      control_duration_(ros::param::param<double>("~control_duration", 0.01)),
      action_client_(ros::param::param<std::string>("~namespaced_action_type", "qbdevice") + "_trajectory_controller/follow_joint_trajectory", false),
      controller_manager_(device.get()) {
  spinner_.start();

  // wait during startup phase to avoid unexpected behaviors
  waitForControllerManager();
  waitForControllers();
  waitForActionServer();
  waitForOtherNodes();

  control_timer_ = node_handle_.createWallTimer(control_duration_, &qbDeviceControl::controlCallback, this);
  ros::Duration(1.0).sleep();  // just to be sure that everything has been startup correctly

  if (ros::param::has("~waypoints")) {
    ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] has waypoints loaded in the parameter server (loop among them).");
    parseWaypoints("~waypoints", waypoint_trajectory_map_);
    move(waypoint_trajectory_map_);
  }
}

qbDeviceControl::~qbDeviceControl() {
  control_timer_.stop();
  spinner_.stop();
}

void qbDeviceControl::actionActiveCallback() {
  //TODO: add sync among control nodes not just at startup
  ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] action start.");
}

void qbDeviceControl::actionDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result) {
  if (result->error_code != result->SUCCESSFUL) {
    ROS_WARN_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] action ended in state [" << state.toString() <<"] with error code [" << result->error_code << "]");
  }
  else {
    ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] action ended in state [" << state.toString() <<"].");
  }

  // automatically loop the waypoint trajectory
  if (ros::param::has("~waypoints")) {
    move(waypoint_trajectory_map_);
  }
}

void qbDeviceControl::actionFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback) {
  ROS_DEBUG_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] current joint state is [" << feedback->actual.positions.at(0) << "] (expecting [" << feedback->desired.positions.at(0) << "]).");
}

void qbDeviceControl::controlCallback(const ros::WallTimerEvent &timer_event) {
  update(timer_event.current_real, timer_event.current_real - timer_event.last_real);
}

void qbDeviceControl::move(const trajectory_msgs::JointTrajectory &joint_trajectory) {
  control_msgs::FollowJointTrajectoryAction control_action;
  control_action.action_goal.goal.trajectory = joint_trajectory;
  control_action.action_goal.header.stamp = ros::Time::now();
  action_client_.sendGoal(control_action.action_goal.goal,
                          std::bind(&qbDeviceControl::actionDoneCallback, this, std::placeholders::_1, std::placeholders::_2),
                          std::bind(&qbDeviceControl::actionActiveCallback, this),
                          std::bind(&qbDeviceControl::actionFeedbackCallback, this, std::placeholders::_1));
}

void qbDeviceControl::move(const std::map<double, std::vector<double>> &waypoints_map) {
  move(setTrajectory(waypoints_map));
}

void qbDeviceControl::parseWaypoints(const std::string &waypoint_map_name, std::map<double, std::vector<double>> &waypoint_map) {
  XmlRpc::XmlRpcValue waypoints;
  if (!ros::param::get(waypoint_map_name, waypoints)) {
    ROS_ERROR_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] fails while retrieving the parameter [" << waypoint_map_name << "].");
    ROS_WARN_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] joint trajectory is expected to be(have been) set from within the code (and remains unchanged).");
    return;  // the 'waypoint_map' remains unchanged
  }
  waypoint_map.clear();
  for (int i=0; i<waypoints.size(); i++) {
    if (!waypoints[i]["joint_positions"].hasMember(device_->getDeviceNamespace())) {
      continue;  // this waypoint does not modify the trajectory of this device (note: be aware of interpolation!)
    }

    // retrieve joint positions for the current device
    std::vector<double> joint_positions;
    for (int j=0; j<waypoints[i]["joint_positions"][device_->getDeviceNamespace()].size(); j++) {
      joint_positions.push_back(xmlCast<double>(waypoints[i]["joint_positions"][device_->getDeviceNamespace()][j]));
    }
    // set joint positions for the whole time interval
    for (int k=0; k<waypoints[i]["time"].size(); k++) {
      waypoint_map.insert(std::make_pair(xmlCast<double>(waypoints[i]["time"][k]), joint_positions));
    }
  }
}

trajectory_msgs::JointTrajectory qbDeviceControl::setTrajectory(const std::map<double, std::vector<double>> &waypoints_map) {
  //TODO: modify 'waypoints_map' to handle velocities and accelerations
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = device_->getJoints();

  for (auto const waypoint : waypoints_map) {
    trajectory_msgs::JointTrajectoryPoint point;
    if (waypoint.second.size() != joint_trajectory.joint_names.size()) {
      ROS_ERROR_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] fails while setting the joint trajectory (joints size mismatch).");
      continue;
    }
    point.positions = waypoint.second;

    for (int j=0; j<waypoint.second.size(); j++) {
      point.velocities.push_back(0);  //FIXME with custom values
    }

    point.time_from_start = ros::Duration(waypoint.first);
    joint_trajectory.points.push_back(point);
  }

  joint_trajectory.header.stamp = ros::Time::now();
  return joint_trajectory;
}

void qbDeviceControl::update(const ros::WallTime& time, const ros::WallDuration& period) {
  // read the state from the hardware
  device_->read(ros::Time(time.toSec()), ros::Duration(period.toSec()));

  // update the controllers (set the new control actions)
  controller_manager_.update(ros::Time(time.toSec()), ros::Duration(period.toSec()));

  // write the commands to the hardware
  device_->write(ros::Time(time.toSec()), ros::Duration(period.toSec()));
}

void qbDeviceControl::waitForActionServer() {
  while (!action_client_.waitForServer(ros::Duration(1.0))) {
    ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] is waiting for the ActionServer to startup...");
  }
}

void qbDeviceControl::waitForControllerManager() {
  ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] is waiting for [controller_manager] to startup...");
  bool load_controller = ros::service::waitForService("controller_manager/load_controller");
  bool switch_controller = ros::service::waitForService("controller_manager/switch_controller");
  bool unload_controller = ros::service::waitForService("controller_manager/unload_controller");
  if (load_controller && switch_controller && unload_controller) {
    ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] [controller_manager] is ready!");
    return;
  }
  ROS_ERROR_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] fails while waiting for [controller_manager].");
}

void qbDeviceControl::waitForControllers() {
  XmlRpc::XmlRpcValue parameters;
  if (!ros::param::get(node_handle_.getNamespace(), parameters)) {
    ROS_ERROR_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] fails while retrieving the parameters.");
    return;
  }
  std::vector<std::string> controllers;
  for (auto param : parameters) {
    std::string param_name(param.first);
    if (std::regex_match(param_name, std::regex("^" + device_->getDeviceNamespace() + "_.*_controller$")) && ros::param::has(param_name + "/type")) {
      controllers.push_back(param_name);
    }
  }
  for (auto controller : controllers) {
    ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] is waiting for controller [" << controller << "] to startup...");
    while (!controller_manager_.getControllerByName(controller)) ;
    ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] controller [" << controller << "] is loaded!");
  }
}

bool qbDeviceControl::waitForResult(const ros::Duration &timeout) {
  return action_client_.waitForResult(timeout);
}

void qbDeviceControl::waitForOtherNodes() {
  ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] is waiting for control nodes synchronization...");
  qb_device_srvs::Trigger srv;
  srv.request.id = device_->getDeviceId();
  sync_nodes_.call(srv);
  while (!srv.response.success) {
    ros::Duration(0.001).sleep();
    sync_nodes_.call(srv);
  }
  ROS_INFO_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] is ready!");
}

template<class T>
T qbDeviceControl::xmlCast(XmlRpc::XmlRpcValue xml_value) {
  // XmlRpcValue does not handle conversion among types but throws an exception if an improper cast is invoked
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    return static_cast<bool>(xml_value);
  }
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(xml_value);
  }
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(xml_value);
  }
  ROS_ERROR_STREAM_NAMED("device_control", "Device [" << device_->getDeviceId() << "] fails while casting the XmlRpcValue [" << xml_value << "].");
  return 0;
}