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

#ifndef QB_DEVICE_JOINT_LIMITS_RESOURCES_H
#define QB_DEVICE_JOINT_LIMITS_RESOURCES_H

namespace qb_device_joint_limits_interface {
/**
 * The qbrobotics Device Joint Limits Resources is a simple class aimed to group all the \p joint_limits_interface::
 * related structures in a unique place. For the sake of uniformity, both \em qbhand and \em qbmove devices exploit the
 * same joint position saturation interface with <b>dynamical management</b> of the joint limits, specifically designed
 * for the \em qbmove shaft limits which vary in relation to the <em>variable stiffness</em> preset. The \em qbhand has
 * fixed joint limits, though.
 * \sa PositionJointSaturationHandle
 * \todo Add soft limits support.
 */
class qbDeviceJointLimitsResources {
 public:
  /**
   * Do nothing.
   * \sa initialize()
   */
  qbDeviceJointLimitsResources()
      : private_node_handle_(ros::NodeHandle("~")) {}

  /**
   * Do nothing.
   */
  virtual ~qbDeviceJointLimitsResources() {}

  /// \name Real-Time Safe Functions
  /**
   * Enforce limits for all managed interfaces.
   * \param period The control period.
   */
  void enforceLimits(const ros::Duration& period) {
    joint_position_saturation_.enforceLimits(period);
    //TODO: add soft limits
  }
  /// \}

  /**
   * Retrieve the joint limits for each given joint. At first they are taken from the \p robot_description (URDF model),
   * but these values are overridden by the one specified in the Parameter Server, if found.
   * \param joints The device HW joints Resource.
   * \param urdf_model The URDF model structure initialized with the \p robot_description.
   * \param joint_position The joint position HW interface, needed to retrieve joint handles.
   * \todo Add soft limits support.
   */
  void initialize(qb_device_hardware_interface::qbDeviceHWResources &joints, const urdf::Model &urdf_model, hardware_interface::PositionJointInterface &joint_position) {
    for (int i=0; i<joints.names.size(); i++) {
      // limits specified in the URDF model overwrite existing values in 'limits' and 'soft_limits'
      // limits specified in the parameter server overwrite existing values in 'limits'
      // limits not specified preserve their existing values
      auto urdf_joint = urdf_model.getJoint(joints.names.at(i));
      const bool urdf_has_limits = joint_limits_interface::getJointLimits(urdf_joint, joints.limits.at(i));
      const bool urdf_has_soft_limits = joint_limits_interface::getSoftJointLimits(urdf_joint, joints.soft_limits.at(i));
      const bool rosparam_has_limits = getJointLimits(joints.names.at(i), private_node_handle_, joints.limits.at(i));

      const hardware_interface::JointHandle joint_handle(joint_position.getHandle(joints.names.at(i)));
      if (urdf_has_soft_limits) {
        //TODO: add soft limits
      }
      else if (urdf_has_limits || rosparam_has_limits) {
        const PositionJointSaturationHandle saturation_handle(joint_handle, &joints.limits.at(i));
        joint_position_saturation_.registerHandle(saturation_handle);
      }
    }
  }

 private:
  ros::NodeHandle private_node_handle_;
  // the position limits interface takes care of the position and velocity limits, but not of the effort ones; however
  // the current is automatically saturated by low level control on the device. The custom interface is aimed to manage
  // the shaft limits which depends on the variable stiffness preset, i.e. they are not a priori fixed.
  PositionJointSaturationInterface joint_position_saturation_;
//  PositionJointSoftLimitsInterface joint_position_soft_limits;  //TODO: actually not yet implemented
};
}  // namespace qb_device_joint_limits_interface

#endif // QB_DEVICE_JOINT_LIMITS_RESOURCES_H