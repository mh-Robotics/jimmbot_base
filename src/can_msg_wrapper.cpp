#include "jimmbot_base/can_msg_wrapper.hpp"

namespace jimmbot_base
{

  void CanMsgWrapper::setSpeed(uint8_t command, double speed)
  {
    this->_command = command;
    this->_speed = speed;
  }

  double CanMsgWrapper::getSpeed(void)
  {
    return this->_speed;
  }

  jimmbot_msgs::CanFrame CanMsgWrapper::getSpeedInCan(void)
  {
    jimmbot_msgs::CanFrame _local;
    _local.id = static_cast<int>(this->_txv_id);
    _local.dlc = CAN_MAX_DLEN;
    _local.data[0] = this->_command;
    _local.data[6] = this->negativeBit(this->_speed);
    _local.data[7] = this->regulateSpeedToUInt8(this->_speed);

    return _local;
  }

  status_t CanMsgWrapper::getStatus(void)
  {
    return UnpackCompressedMotorStatus(this->_status_frame);
  }

  jimmbot_msgs::CanFrame CanMsgWrapper::getLightsInCan(const std::pair<bool, bool> &lights)
  {
    jimmbot_msgs::CanFrame _local;
    _local.id = static_cast<int>(0x31);
    _local.dlc = CAN_MAX_DLEN;
    _local.data[static_cast<int>(CanMsgWrapper::LightsId::ENABLE_LIGHT_LEFT_MSG_INDEX)] = lights.first;
    _local.data[static_cast<int>(CanMsgWrapper::LightsId::ENABLE_LIGHT_RIGHT_MSG_INDEX)] = lights.second;

    return _local;
  }

  void CanMsgWrapper::updateStatusFrame(jimmbot_msgs::CanFrame status_frame)
  {
    this->_status_frame = status_frame;
  }

  // Pack a motor status struct into a compressed CAN frame
  jimmbot_msgs::CanFrame CanMsgWrapper::PackCompressedMotorStatus(const status_t& motorStatus) {
    jimmbot_msgs::CanFrame canFrame;
    canFrame.id = motorStatus._id;
    canFrame.dlc = CAN_MAX_DLEN;

    // Compress the motor status data into a bit-field struct
    compressed_motor_status_t compressedStatus;
    compressedStatus.command_id = motorStatus._command;
    compressedStatus.effort = motorStatus._effort;
    compressedStatus.position = static_cast<uint32_t>(motorStatus._position * 100);
    compressedStatus.rpm = motorStatus._rpm;
    compressedStatus.velocity = static_cast<uint32_t>(motorStatus._velocity * 100);
    compressedStatus.can_id = motorStatus._id & 0x7FF;

    // Copy the compressed data into the CAN frame
    std::memcpy(canFrame.data.c_array(), &compressedStatus, sizeof(compressed_motor_status_t));

    return canFrame;
  }

  // Unpack a motor status struct from a compressed CAN frame
  status_t CanMsgWrapper::UnpackCompressedMotorStatus(const jimmbot_msgs::CanFrame& canFrame) {
    status_t motorStatus;

    // Extract the compressed data from the CAN frame
    compressed_motor_status_t compressedStatus;
    std::memcpy(&compressedStatus, canFrame.data.data(), sizeof(compressed_motor_status_t));

    // Unpack the compressed data into the motor status struct
    motorStatus._id = canFrame.id;
    motorStatus._command = compressedStatus.command_id;
    motorStatus._effort = compressedStatus.effort;
    motorStatus._position = static_cast<double>(compressedStatus.position) / 100;
    motorStatus._rpm = compressedStatus.rpm;
    motorStatus._velocity = static_cast<double>(compressedStatus.velocity) / 100;

    return motorStatus;
  }

  bool CanMsgWrapper::negativeBit(const double &speed) const
  {
    return speed < 0 ? true : false;
  }

  double CanMsgWrapper::linearToAngular(const double &speed) const
  {
    return speed / this->getWheelDiameter() * 2;
  }

  double CanMsgWrapper::angularToLinear(const double &speed) const
  {
    return speed * this->getWheelDiameter() / 2;
  }

  uint8_t CanMsgWrapper::regulateSpeedToUInt8(double speed)
  {
    return (std::abs(this->angularToLinear(speed)) * UINT8_MAX / this->getMaxSpeed());
  }

  double CanMsgWrapper::regulateSpeedToDouble(uint8_t first_byte, uint8_t second_byte)
  {
    return (((second_byte / 10) + first_byte) * 10);
  }

  double CanMsgWrapper::regulatePositionToDouble(uint8_t first_byte, uint8_t second_byte)
  {
    return (((second_byte / 10) + first_byte) * 10);
  }

  int CanMsgWrapper::regulateRpmToInt(uint8_t first_byte, uint8_t second_byte)
  {
    return (((second_byte / 10) + first_byte) * 10);
  }

  void CanMsgWrapperCommand::execute() 
  {
    auto _pair = type2func.find(_command);

    if(_pair != type2func.end())
    {
      _pair->second();
    }
  }

}//end namespace jimmbot_base