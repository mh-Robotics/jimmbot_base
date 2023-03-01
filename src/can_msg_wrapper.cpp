#include "jimmbot_base/can_msg_wrapper.hpp"

namespace jimmbot_base {

void CanMsgWrapper::setSpeed(uint8_t command, double speed) {
  command_ = command;
  speed_ = speed;
}

double CanMsgWrapper::getSpeed(void) { return speed_; }

jimmbot_msgs::CanFrame CanMsgWrapper::getSpeedInCan(void) {
  jimmbot_msgs::CanFrame _local;
  _local.id = static_cast<int>(canpressor_->ReceiveId());
  _local.dlc = CAN_MAX_DLEN;
  _local.data[0] = command_;
  _local.data[6] = negativeBit(speed_);
  _local.data[7] = regulateSpeedToUInt8(speed_);

  return _local;
}

wheel_status_t CanMsgWrapper::getStatus(void) {
  return canpressor_->UnpackCompressed<jimmbot_msgs::CanFrame, wheel_status_t>(
      status_frame_);
}

jimmbot_msgs::CanFrame CanMsgWrapper::getLightsInCan(
    const std::pair<bool, bool>& lights) {
  jimmbot_msgs::CanFrame _local;
  _local.id = static_cast<int>(0x31);
  _local.dlc = CAN_MAX_DLEN;
  _local.data[static_cast<int>(CanMsgWrapper::LightsId::kLeftLightEnableBit)] =
      lights.first;
  _local.data[static_cast<int>(CanMsgWrapper::LightsId::kRightLightEnableBit)] =
      lights.second;

  return _local;
}

void CanMsgWrapper::updateStatusFrame(jimmbot_msgs::CanFrame status_frame) {
  status_frame_ = status_frame;
}

bool CanMsgWrapper::negativeBit(const double& speed) const {
  return speed < 0 ? true : false;
}

double CanMsgWrapper::linearToAngular(const double& speed) const {
  return speed / getWheelDiameter() * 2;
}

double CanMsgWrapper::angularToLinear(const double& speed) const {
  return speed * getWheelDiameter() / 2;
}

uint8_t CanMsgWrapper::regulateSpeedToUInt8(double speed) {
  return (std::abs(angularToLinear(speed)) * UINT8_MAX / getMaxSpeed());
}

double CanMsgWrapper::regulateSpeedToDouble(uint8_t first_byte,
                                            uint8_t second_byte) {
  return (((second_byte / 10) + first_byte) * 10);
}

double CanMsgWrapper::regulatePositionToDouble(uint8_t first_byte,
                                               uint8_t second_byte) {
  return (((second_byte / 10) + first_byte) * 10);
}

int CanMsgWrapper::regulateRpmToInt(uint8_t first_byte, uint8_t second_byte) {
  return (((second_byte / 10) + first_byte) * 10);
}

void CanMsgWrapperCommand::execute() {
  auto pair = type2func.find(command_);

  if (pair != type2func.end()) {
    pair->second();
  }
}

}  // end namespace jimmbot_base