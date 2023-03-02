/**
 * @file jimmbot_hardware_interface.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file contains the declaration of the JimmBotHardwareInterface
 * class. The JimmBotHardwareInterface is a class that implements the ROS
 * RobotHW interface. It provides an interface to interact with the hardware of
 * the JimmBot robot.
 * @version 0.1
 * @date 2021-03-23
 *
 * @copyright Copyright (c) 2020-2023, mhRobotics, Inc. All rights reserved.
 * @license This project is released under the BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef JIMMBOT_HARDWARE_INTERFACE_H_
#define JIMMBOT_HARDWARE_INTERFACE_H_

#include <hardware_interface/joint_command_interface.h>  // for hardware_interface::VelocityJointInterface
#include <hardware_interface/joint_state_interface.h>  // for hardware_interface::JointStateInterface
#include <hardware_interface/robot_hw.h>   // for hardware_interface::RobotHW
#include <jimmbot_msgs/CanFrameStamped.h>  // for jimmbot_msgs::CanFrameStamped
#include <jimmbot_msgs/ExtnDataStamped.h>  // for jimmbot_msgs::ExtnDataStamped
#include <ros/node_handle.h>               // for ros::NodeHandle
#include <ros/publisher.h>                 // for ros::Publisher
#include <ros/subscriber.h>                // for ros::Subscriber
#include <std_msgs/Float64.h>              // for std_msgs::Float64

#include "can_msg_wrapper.hpp"       // for CanMsgWrapper
#include "jimmbot_base/constants.h"  // for jimmbot_base::k*

/**
 * @brief jimmbot_base namespace
 *
 */
namespace jimmbot_base {
/**
 * @brief Returns a lambda function that calculates the index of the iterator in
 * the given collection.
 *
 * @tparam Collection The type of the collection.
 * @param collection The collection.
 * @param offset The offset to add to the index.
 * @return The lambda function that calculates the index.
 */
template <typename Collection>
auto GetIndex(Collection const& collection, size_t offset = 0) {
  return [&collection, offset](auto const& iterator) {
    return offset + std::distance(std::begin(collection), iterator);
  };
}

/**
 * @brief Definition of the JimmBotHardwareInterface class.
 *
 * The JimmBotHardwareInterface is a class that implements the ROS RobotHW
 * interface. It provides an interface to interact with the hardware of the
 * JimmBot robot.
 */
class JimmBotHardwareInterface : public hardware_interface::RobotHW {
 public:
  /**
   * @brief The Joint element structure.This struct is used to save the data for
   * a joint.
   */
  using JointElements = struct JointElements {
    mutable WheelStatus command;   ///< The command wheel status.
    mutable WheelStatus feedback;  ///< The feedback wheel status.
  };

  /**
   * @brief Construct a new Jimm Bot Hardware Interface object
   *
   */
  JimmBotHardwareInterface() = delete;

  /**
   * @brief Constructs a new JimmBotHardwareInterface object.
   *
   * @param nh A pointer to the ROS node handle.
   * @param nh_param A pointer to the ROS node handle for parameters.
   */
  JimmBotHardwareInterface(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

  /**
   * @brief Register the controller interface
   */
  void RegisterControlInterfaces();

  /**
   * @brief Read data from hardware and update the state
   */
  void read(const ros::Time& /*time*/,
            const ros::Duration& /*period*/) override;

  /**
   * @brief Write data to the hardware
   */
  void write(const ros::Time& /*time*/,
             const ros::Duration& /*period*/) override;

  /**
   * @brief Returns the current time.
   *
   * @return The current time.
   */
  [[nodiscard]] inline ros::Time GetTimeNow() const { return ros::Time::now(); }

  /**
   * @brief Returns the control frequency.
   *
   * @return The control frequency.
   */
  [[nodiscard]] inline double GetControlFrequency() const {
    return this->control_frequency_;
  }

  /**
   * @brief Returns the elapsed time since the last update.
   *
   * @param last_time The last update time.
   * @return The elapsed time.
   */
  [[nodiscard]] inline ros::Duration GetElapsedTime(ros::Time last_time) const {
    return static_cast<ros::Duration>(this->GetTimeNow() - last_time);
  }

  /**
   * @brief The callback function for the CAN feedback message.
   *
   * @param feedback_msg The feedback message.
   */
  void CanFeedbackMsgCallback(
      const jimmbot_msgs::CanFrameStamped::ConstPtr& feedback_msg);

  /**
   * @brief Callback function for external data message.
   *
   * @param extn_data_msg Pointer to the external data message.
   */
  void ExtnDataMsgCallback(
      const jimmbot_msgs::ExtnDataStamped::ConstPtr& extn_data_msg);

  /**
   * @brief Callback function for front camera tilt angle.
   *
   * @param angle Pointer to the front camera tilt angle.
   */
  void CameraTiltFrontCallback(const std_msgs::Float64::ConstPtr& angle);

  /**
   * @brief Callback function for back camera tilt angle.
   *
   * @param angle Pointer to the back camera tilt angle.
   */
  void CameraTiltBackCallback(const std_msgs::Float64::ConstPtr& angle);

 private:
  /**
   * @brief Update the joints from hardware.
   */
  void UpdateJointsFromHardware() const;

  /**
   * @brief Update the speed to hardware.
   */
  void UpdateSpeedToHardware() const;

  /**
   * @brief Update the angle to kinect cameras.
   */
  void UpdateAngleToKinectCameras();

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;

  std::vector<JointElements> joint_elements_;

  CanMsgWrapper front_left_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelFrontLeft),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelFrontLeft));
  CanMsgWrapper front_right_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelFrontRight),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelFrontRight));
  CanMsgWrapper back_left_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelBackLeft),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelBackLeft));
  CanMsgWrapper back_right_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelBackRight),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelBackRight));

  ros::NodeHandle nh_;
  ros::Publisher esp32_can_pub_;
  ros::Subscriber esp32_can_sub_;
  ros::Subscriber extn_data_sub_;
  std::pair<ros::Subscriber, ros::Subscriber> camera_tilt_sub_;
  std::pair<bool, bool> lights_;
  std::pair<float, float> camera_angles_;

  double control_frequency_;
  double max_wheel_speed_;
  std::string frame_id_;

  std::string left_wheel_front_;
  std::string left_wheel_back_;
  std::string right_wheel_front_;
  std::string right_wheel_back_;
};
}  // end namespace jimmbot_base
#endif  // end JIMMBOT_HARDWARE_INTERFACE_H_
