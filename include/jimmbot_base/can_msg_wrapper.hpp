/**
 * @file can_msg_wrapper.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file contains the definition of the CanMsgWrapper class.
 * CanMsgWrapper is a class that allows for the easy exchange of information
 * with the hardware of the JimmBot robot using the CAN (Controller Area
 * Network) protocol. The class encapsulates a CAN message and provides methods
 * for setting and getting the message data, such as the ID, length, and
 * payload.
 * @version 0.2
 * @date 2023-03-03
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
#ifndef JIMMBOT_BASE_CAN_MSG_WRAPPER_H_
#define JIMMBOT_BASE_CAN_MSG_WRAPPER_H_

#include <jimmbot_msgs/CanFrame.h>         // for jimmbot_msg::CanFrame
#include <jimmbot_msgs/CanFrameStamped.h>  // for jimmbot_msg::CanFrameStamped

#include <mutex>          // for std::mutex
#include <unordered_map>  // for std::unordered_map

#include "can_packt.h"  // for CanPackt

namespace jimmbot_base {

constexpr auto kWheelDiameter = 0.1651;

std::ostream& operator<<(std::ostream& os,
                         const jimmbot_msgs::CanFrame::ConstPtr& obj) {
  os << "Data length: " << std::hex << static_cast<double>(obj->dlc)
     << std::endl;

  os << "Data: " << static_cast<double>(obj->data[0]) << "  "
     << static_cast<double>(obj->data[1]) << "  "
     << static_cast<double>(obj->data[2]) << "  "
     << static_cast<double>(obj->data[3]) << "  "
     << static_cast<double>(obj->data[4]) << "  "
     << static_cast<double>(obj->data[5]) << "  "
     << static_cast<double>(obj->data[6]) << "  "
     << static_cast<double>(obj->data[7]) << std::endl;

  return os;
}

std::ostream& operator<<(std::ostream& os, const WheelStatus& obj) {
  os << "Command: " << obj.command_id << ", Effort: " << obj.effort
     << ", Position: " << obj.position << ", RPM: " << obj.rpm
     << ", Velocity: " << obj.velocity << std::endl;

  return os;
}

class CanMsgWrapper {
  /**
   * @brief An enum representing the different CAN IDs
   */
  // @todo(issues/5): Try to convert the CanID enum to private.
 public:
  enum class CanId : uint8_t {
    kBegin = 0,
    kCommandWheelFrontLeft = kBegin,
    kCommandWheelFrontRight,
    kCommandWheelBackLeft,
    kCommandWheelBackRight,
    kFeedbackWheelFrontLeft,
    kFeedbackWheelFrontRight,
    kFeedbackWheelBackLeft,
    kFeedbackWheelBackRight,
    kUnspecified,
    kEnd
  };

 public:
  enum class LightsId : uint8_t {
    kLeftLightEnableBit = 3,
    kRightLightEnableBit = 7
  };

  /**
   * @brief Construct a new Can Msg Wrapper object
   *
   * @param transmit_id
   * @param receive_id
   */
  CanMsgWrapper(uint8_t transmit_id, uint8_t receive_id)
      : canpressor_(std::make_unique<CanPackt>(transmit_id, receive_id)){};

  /**
   * @brief Set the Speed object
   *
   * @param command
   * @param speed
   */
  void SetWheelCommandStatus(const WheelStatus& command_status);

  /**
   * @brief Get the Speed In Can object
   *
   * @return jimmbot_msgs::CanFrame
   */
  [[nodiscard]] jimmbot_msgs::CanFrame GetWheelCommandStatus() const;

  /**
   * @brief Get the Status object
   *
   * @return WheelStatus
   */
  [[nodiscard]] WheelStatus GetWheelFeedbackStatus() const;

  /**
   * @brief Get the Lights In Can object
   *
   * @param lights
   * @return jimmbot_msgs::CanFrame
   */
  [[nodiscard]] static jimmbot_msgs::CanFrame GetLightsInCan(
      const std::pair<bool, bool>& lights);

  /**
   * @brief
   *
   * @param status_frame
   */
  void UpdateWheelFeedbackStatusFrame(jimmbot_msgs::CanFrame status_frame);

  [[nodiscard]] inline uint8_t TransmitId() const {
    return canpressor_->TransmitId();
  };
  [[nodiscard]] inline uint8_t ReceiveId() const {
    return canpressor_->ReceiveId();
  };

  /**
   * @brief Convert linear travel in meters to angular rotation in radians
   *
   * @param travel The linear travel to convert, in meters
   * @return The equivalent angular rotation, in radians
   */
  [[nodiscard]] inline double LinearToAngular(const double& travel) const {
    return travel / kWheelDiameter * 2;
  };

  /**
   * @brief Convert angular rotation in radians to linear travel in meters
   *
   * @param angle The angular rotation to convert, in radians
   * @return The equivalent linear travel, in meters
   */
  [[nodiscard]] inline double AngularToLinear(const double& angle) const {
    return angle * kWheelDiameter / 2;
  };

 private:
  std::unique_ptr<CanPackt> canpressor_;
  jimmbot_msgs::CanFrame feedback_status_;
  mutable std::mutex feedback_mutex_;
  WheelStatus command_status_;
  mutable std::mutex command_mutex_;
};

class Command {
  virtual void Execute() = 0;
};

class CanMsgWrapperCommand : Command {
  /**
   * @brief An enum representing the different commands
   */
  // @todo(issues/5): Try to convert the Command enum to private.
 public:
  enum class Command : uint8_t {
    kBegin = 0,
    kWheelOff = kBegin,
    kWheelOn,
    kWheelStatus,
    kWheelStatusUpdate,
    kWheelSpeed,
    kUnspecified,
    kEnd
  };

 public:
  CanMsgWrapper& can_wrapper;
  Command command;
  jimmbot_msgs::CanFrame feedback_status;
  WheelStatus command_status;

  CanMsgWrapperCommand(const CanMsgWrapperCommand&) = default;
  CanMsgWrapperCommand(CanMsgWrapperCommand&&) = default;
  CanMsgWrapperCommand& operator=(const CanMsgWrapperCommand&) = delete;
  CanMsgWrapperCommand& operator=(CanMsgWrapperCommand&&) = delete;

  CanMsgWrapperCommand(std::reference_wrapper<CanMsgWrapper> can_wrapper,
                       Command command, WheelStatus wheel_status)
      : can_wrapper(can_wrapper),
        command(command),
        command_status(wheel_status) {}

  CanMsgWrapperCommand(std::reference_wrapper<CanMsgWrapper> can_wrapper,
                       Command command, jimmbot_msgs::CanFrame can_frame)
      : can_wrapper(can_wrapper),
        command(command),
        feedback_status(can_frame) {}

  CanMsgWrapperCommand(std::reference_wrapper<CanMsgWrapper> can_wrapper,
                       Command command)
      : can_wrapper(can_wrapper), command(command) {}

  std::unordered_map<Command, const std::function<void()>> type2func{
      {Command::kWheelOff,
       [&]() { return can_wrapper.SetWheelCommandStatus(command_status); }},
      {Command::kWheelOn,
       [&]() { return can_wrapper.SetWheelCommandStatus(command_status); }},
      {Command::kWheelStatus,
       [&]() { return can_wrapper.GetWheelFeedbackStatus(); }},
      {Command::kWheelStatusUpdate,
       [&]() {
         return can_wrapper.UpdateWheelFeedbackStatusFrame(feedback_status);
       }},
      {Command::kWheelSpeed,
       [&]() { return can_wrapper.SetWheelCommandStatus(command_status); }}};

  void Execute() override;
};

}  // end namespace jimmbot_base
#endif  // end JIMMBOT_BASE_CAN_MSG_WRAPPER_H_
