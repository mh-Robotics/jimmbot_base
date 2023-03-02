#include "jimmbot_base/can_msg_wrapper.hpp"  // for CanMsgWrapper

namespace jimmbot_base {

void CanMsgWrapper::SetWheelCommandStatus(const WheelStatus& command_status) {
  command_status_ = command_status;
}

jimmbot_msgs::CanFrame CanMsgWrapper::GetWheelCommandStatus() const {
  return canpressor_->PackCompressed<WheelStatus, jimmbot_msgs::CanFrame>(
      command_status_);
}

WheelStatus CanMsgWrapper::GetWheelFeedbackStatus() const {
  return canpressor_->UnpackCompressed<jimmbot_msgs::CanFrame, WheelStatus>(
      feedback_status_);
}

jimmbot_msgs::CanFrame CanMsgWrapper::GetLightsInCan(
    const std::pair<bool, bool>& lights) {
  jimmbot_msgs::CanFrame local;
  local.id = static_cast<int>(0x31);
  local.dlc = CAN_MAX_DLEN;
  local.data[static_cast<int>(CanMsgWrapper::LightsId::kLeftLightEnableBit)] =
      static_cast<unsigned char>(lights.first);
  local.data[static_cast<int>(CanMsgWrapper::LightsId::kRightLightEnableBit)] =
      static_cast<unsigned char>(lights.second);

  return local;
}

void CanMsgWrapper::UpdateWheelFeedbackStatusFrame(
    jimmbot_msgs::CanFrame status_frame) {
  feedback_status_ = status_frame;
}

void CanMsgWrapperCommand::Execute() {
  auto pair = type2func.find(command);

  if (pair != type2func.end()) {
    pair->second();
  }
}

}  // end namespace jimmbot_base
