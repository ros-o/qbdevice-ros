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

#ifndef QB_DEVICE_JOINT_LIMITS_INTERFACE_H
#define QB_DEVICE_JOINT_LIMITS_INTERFACE_H

namespace qb_device_joint_limits_interface {
/**
 * A handle used to enforce position and velocity limits of a position-controlled joint that does not have soft limits.
 * The only difference from joint_limits_interface::PositionJointSaturationHandle is that this exploit a <b>dynamical
 * management</b> of the joint limits, i.e. the limits can be modified at runtime by others.
 */
class PositionJointSaturationHandle {
 public:
  /**
   * Construct the saturation handle with \b dynamic joint limits.
   * \param joint_handle The joint handle of interest.
   * \param limits The \p pointer to the joint limits structure of the joint.
   */
  PositionJointSaturationHandle(const hardware_interface::JointHandle &joint_handle, joint_limits_interface::JointLimits* limits)
      : joint_handle_(joint_handle),
        limits_(limits),
        command_old_(joint_handle_.getPosition()) {
    if (!limits_->has_position_limits) {
      limits_->min_position = -std::numeric_limits<double>::max();
      limits_->max_position = std::numeric_limits<double>::max();
    }
  }

  /**
   * Enforce position and velocity limits for a joint that is not subject to soft limits. \b Note: be aware of dynamic
   * joint limits.
   * \param period The control period.
   */
  void enforceLimits(const ros::Duration& period) {
    double min_pos, max_pos;
    if (limits_->has_velocity_limits) {
      const double delta_pos = limits_->max_velocity * period.toSec();
      min_pos = std::max(command_old_ - delta_pos, limits_->min_position);
      max_pos = std::min(command_old_ + delta_pos, limits_->max_position);
    }
    else {
      min_pos = limits_->min_position;
      max_pos = limits_->max_position;
    }

    const double command = joint_limits_interface::internal::saturate(joint_handle_.getCommand(), min_pos, max_pos);
    joint_handle_.setCommand(command);
    command_old_ = command;
  }

  /**
   * \return The joint name.
   */
  std::string getName() const { return joint_handle_.getName(); }

  /**
   * Reset the interface state.
   */
  void reset() { command_old_ = joint_handle_.getPosition(); }

 private:
  hardware_interface::JointHandle joint_handle_;
  joint_limits_interface::JointLimits* limits_;
  double command_old_;
};

/**
 * Interface for enforcing limits on a position-controlled joint through saturation. The only difference from
 * joint_limits_interface::PositionJointSaturationInterface is that this exploit a <b>dynamical management</b> of the
 * joint limits, i.e. the limits can be modified at runtime by others.
 */
class PositionJointSaturationInterface : public joint_limits_interface::JointLimitsInterface<PositionJointSaturationHandle> {
 public:
  /// \name Real-Time Safe Functions
  /**
   * Reset all managed handles.
   */
  void reset() {
    typedef hardware_interface::ResourceManager<PositionJointSaturationHandle>::ResourceMap::iterator ItratorType;
    for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it) {
      it->second.reset();
    }
  }
  /// \}
};
}  // namespace qb_device_joint_limits_interface

#endif // QB_DEVICE_JOINT_LIMITS_INTERFACE_H