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

  jimmbot_msgs::canFrame CanMsgWrapper::getSpeedInCan(void)
  {
    jimmbot_msgs::canFrame _local;
    _local.id = static_cast<int>(this->_txv_id);
    _local.dlc = 0x8;
    _local.data[0] = this->_command;
    _local.data[6] = this->negativeBit(this->_speed);
    _local.data[7] = this->regulateSpeedToUInt8(this->_speed);

    return _local;
  }

  status_t CanMsgWrapper::getStatus(void)
  {
    status_t _local;
    _local._id = static_cast<int>(this->_rxv_id);
    _local._command = this->_status_frame.data[0];
    _local._effort = (this->_status_frame.data[1] / 10);
    _local._position = this->regulatePositionToDouble(this->_status_frame.data[2], this->_status_frame.data[3]);
    _local._rpm = this->regulateRpmToInt(this->_status_frame.data[4], this->_status_frame.data[5]);
    _local._speed = this->regulateSpeedToDouble(this->_status_frame.data[6], this->_status_frame.data[7]);

    return _local;
  }

  jimmbot_msgs::canFrame CanMsgWrapper::getLightsInCan(const std::pair<bool, bool> &lights)
  {
    jimmbot_msgs::canFrame _local;
    _local.id = static_cast<int>(0x31);
    _local.dlc = 0x8;
    _local.data[static_cast<int>(CanMsgWrapper::LightsId::ENABLE_LIGHT_LEFT_MSG_INDEX)] = lights.first;
    _local.data[static_cast<int>(CanMsgWrapper::LightsId::ENABLE_LIGHT_RIGHT_MSG_INDEX)] = lights.second;

    return _local;
  }

  void CanMsgWrapper::updateStatusFrame(jimmbot_msgs::canFrame status_frame)
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