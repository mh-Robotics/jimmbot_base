/**
 * @file can_msg_wrapper.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-03-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ___CAN_MSG_WRAPPER___
#define ___CAN_MSG_WRAPPER___

#include <ros/ros.h>
#include <jimmbot_msgs/canFrame.h>
#include <jimmbot_msgs/canFrameArray.h>

#include <string>
#include <unordered_map>
#include <cstdint>

namespace jimmbot_base
{
  std::ostream &operator << (std::ostream &os, const jimmbot_msgs::canFrame::ConstPtr &obj)
  {
    os << "Id: " << std::hex << "0x" << obj->id << ", Data length: " << std::hex << static_cast<double>(obj->dlc) << std::endl;

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

  typedef struct Status
  {
    int _id = {0};
    int _command = {0};
    double _effort = {0};
    double _position = {0};
    int _rpm = {0};
    double _speed = {0};
  }status_t;

  std::ostream &operator << (std::ostream &os, const status_t &obj)
  {
    os << "Id: " << std::hex << "0x" << obj._id
       << ", Command: " << obj._command
       << ", Effort: " << obj._effort
       << ", Position: " << obj._position
       << ", RPM: " << obj._rpm
       << ", Speed: " << obj._speed << std::endl;

    return os;
  }

  class CanMsgWrapper
  {
    public:
    enum class CanId : uint8_t 
    { 
      COMMAND_WHEEL_FRONT_LEFT = 0x00,
      COMMAND_WHEEL_FRONT_RIGHT,
      COMMAND_WHEEL_BACK_LEFT,
      COMMAND_WHEEL_BACK_RIGHT,
      FEEDBACK_WHEEL_FRONT_LEFT,
      FEEDBACK_WHEEL_FRONT_RIGHT,
      FEEDBACK_WHEEL_BACK_LEFT,
      FEEDBACK_WHEEL_BACK_RIGHT,
    };

    public:
      /**
       * @brief Construct a new Can Msg Wrapper object
       * 
       * @param transmit_id 
       * @param receive_id 
       */
      CanMsgWrapper(CanId transmit_id, CanId receive_id) : _txv_id(transmit_id),
                                                           _rxv_id(receive_id) { };

      /**
       * @brief Set the Speed object
       * 
       * @param command 
       * @param speed 
       */
      void setSpeed(uint8_t command, double speed);

      /**
       * @brief Get the Speed object
       * 
       * @return double 
       */
      double getSpeed(void);

      /**
       * @brief Get the Speed In Can object
       * 
       * @return jimmbot_msgs::canFrame 
       */
      jimmbot_msgs::canFrame getSpeedInCan(void);

      /**
       * @brief Get the Status object
       * 
       *  data[0] = static_cast<uint8_t>(CanWrapper::Command::MOTOR_STATUS);
       *  data[1] = static_cast<uint8_t>(this->_wheel_controller.getWheelEffort()); //real number, divide by 10, 25 = 2.5
       *  data[2] = static_cast<uint8_t>(this->_wheel_controller.getWheelPosition()); //25.5 truncated to 25
       *  data[3] = static_cast<uint8_t>(this->_wheel_controller.getWheelPosition() - data[3] * 10); //25.5 - 20 = 0.5 * 10 = 5
       *  data[4] = static_cast<uint8_t>(this->_wheel_controller.getWheelRpm() / 10); //455 / 10 = 45.5 truncated to 45 
       *  data[5] = static_cast<uint8_t>(this->_wheel_controller.getWheelRpm() - data[4]); //45.5 - 45 = 0.5 * 10 = 5
       *  data[6] = static_cast<uint8_t>(this->_wheel_controller.getWheelVelocity()); //2.5 truncated to 2
       *  data[7] = static_cast<uint8_t>((this->_wheel_controller.getWheelVelocity() - data[6]) * 10); //2.5 - 2 = 0.5 * 10 = 5
       * 
       * @return status_t 
       */
      status_t getStatus(void);

      /**
       * @brief 
       * 
       * @param status_frame 
       */
      void updateStatusFrame(jimmbot_msgs::canFrame status_frame);

    private:
      /**
       * @brief 
       * 
       * @param speed 
       * @return true 
       * @return false 
       */
      bool negativeBit(double speed);

      /**
       * @brief 
       * 
       * @param speed 
       * @return uint8_t 
       */
      uint8_t regulateSpeedToUInt8(double speed);

      /**
       * @brief 
       * 
       * @param first_byte 
       * @param second_byte 
       * @return double 
       */
      double regulateSpeedToDouble(uint8_t first_byte, uint8_t second_byte);

      /**
       * @brief 
       * 
       * @param first_byte 
       * @param second_byte 
       * @return double 
       */
      double regulatePositionToDouble(uint8_t first_byte, uint8_t second_byte);

      /**
       * @brief 
       * 
       * @param first_byte 
       * @param second_byte 
       * @return int 
       */
      int regulateRpmToInt(uint8_t first_byte, uint8_t second_byte);

      CanId _rxv_id;
      CanId _txv_id;
      jimmbot_msgs::canFrame _status_frame;
      double _speed;
      uint8_t _command;
      double _max_speed{2.5};
  };

  class Command 
  {
    virtual void execute() = 0;
  };

  class CanMsgWrapperCommand : Command 
  {
    public:
    enum class Command : uint8_t 
    { 
      BEGIN = 0,
      MOTOR_OFF = BEGIN,
      MOTOR_ON,
      MOTOR_STATUS,
      MOTOR_STATUS_UPDATE,
      MOTOR_SPEED,
      UNSPECIFIED,
      END
    };

    CanMsgWrapper& _can_wrapper;
    Command _command;
    double _value;
    jimmbot_msgs::canFrame _status_frame;

    CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command, double value) : _can_wrapper(can_wrapper), 
                                                                                      _command(command),
                                                                                      _value(value) { }

    CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command, jimmbot_msgs::canFrame status_frame) : _can_wrapper(can_wrapper), 
                                                                                                             _command(command),
                                                                                                             _status_frame(status_frame) { }

    CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command) : _can_wrapper(can_wrapper), 
                                                                        _command(command) { }

    std::unordered_map<Command, std::function<void()>> type2func
    {
      {Command::MOTOR_OFF,           [&](){ return this->_can_wrapper.setSpeed(static_cast<uint8_t>(Command::MOTOR_SPEED), 0.0); }},
      {Command::MOTOR_ON,            [&](){ return this->_can_wrapper.setSpeed(static_cast<uint8_t>(Command::MOTOR_SPEED), 0.0); }},
      {Command::MOTOR_STATUS,        [&](){ return this->_can_wrapper.getStatus(); }},
      {Command::MOTOR_STATUS_UPDATE, [&](){ return this->_can_wrapper.updateStatusFrame(_status_frame); }},
      {Command::MOTOR_SPEED,         [&](){ return this->_can_wrapper.setSpeed(static_cast<uint8_t>(Command::MOTOR_SPEED), _value); }}
    };

    void execute();
  };

}//end namespace jimmbot_base
#endif//end ___CAN_MSG_WRAPPER___