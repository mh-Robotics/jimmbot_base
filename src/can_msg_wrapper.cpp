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

  wheel_status_t CanMsgWrapper::getStatus(void)
  {
    return canpressor_->UnpackCompressed<jimmbot_msgs::CanFrame, wheel_status_t>(this->_status_frame);
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