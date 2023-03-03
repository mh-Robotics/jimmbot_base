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
#include "jimmbot_base/can_msg_wrapper.hpp"  // for CanMsgWrapper

namespace jimmbot_base {

void CanMsgWrapper::SetWheelCommandStatus(const WheelStatus& command_status) {
  command_status_ = command_status;
}

jimmbot_msgs::CanFrame CanMsgWrapper::GetWheelCommandStatus() const {
  // @todo(issues/3): Add a scoped_mutex for command_status_
  return canpressor_->PackCompressed<WheelStatus, jimmbot_msgs::CanFrame>(
      command_status_);
}

WheelStatus CanMsgWrapper::GetWheelFeedbackStatus() const {
  // @todo(issues/4): Add a scoped_mutex for feedback_status_
  return canpressor_->UnpackCompressed<jimmbot_msgs::CanFrame, WheelStatus>(
      feedback_status_);
}

jimmbot_msgs::CanFrame CanMsgWrapper::GetLightsInCan(
    const std::pair<bool, bool>& lights) {
  jimmbot_msgs::CanFrame local;
  local.id = jimmbot_base::kLightCanMsgId;
  local.dlc = jimmbot_base::kCanMaxDLen;
  local.data[static_cast<int>(CanMsgWrapper::LightsId::kLeftLightEnableBit)] =
      static_cast<unsigned char>(lights.first);
  local.data[static_cast<int>(CanMsgWrapper::LightsId::kRightLightEnableBit)] =
      static_cast<unsigned char>(lights.second);

  return local;
}

void CanMsgWrapper::UpdateWheelFeedbackStatusFrame(
    jimmbot_msgs::CanFrame status_frame) {
  if (status_frame.id == canpressor_->ReceiveId()) {
    feedback_status_ = status_frame;
  }
}

void CanMsgWrapperCommand::Execute() {
  auto pair = type2func.find(command);

  if (pair != type2func.end()) {
    pair->second();
  }
}

}  // end namespace jimmbot_base
