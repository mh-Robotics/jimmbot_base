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
#include <jimmbot_msgs/CanFrame.h> // for jimmbot_msg::CanFrame
#include <jimmbot_msgs/CanFrameStamped.h> // for jimmbot_msg::CanFrameStamped

#include <string>
#include <unordered_map>
#include <cstdint>

#define CAN_MAX_DLEN 8

namespace jimmbot_base
{
  std::ostream &operator << (std::ostream &os, const jimmbot_msgs::CanFrame::ConstPtr &obj)
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
    int _effort = {0};
    double _position = {0};
    int _rpm = {0};
    double _velocity = {0};
  }status_t;

  // Define a bit-field struct to represent the compressed motor status data
typedef struct __attribute__((packed)) {
    uint32_t can_id : 11;
    uint32_t command_id : 8;
    uint32_t effort : 12;
    uint32_t position : 20;
    uint32_t rpm : 10;
    uint32_t velocity : 24;
} compressed_motor_status_t;

  std::ostream &operator << (std::ostream &os, const status_t &obj)
  {
    os << "Id: " << std::hex << "0x" << obj._id
       << ", Command: " << obj._command
       << ", Effort: " << obj._effort
       << ", Position: " << obj._position
       << ", RPM: " << obj._rpm
       << ", Velocity: " << obj._velocity << std::endl;

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

    enum class LightsId : uint8_t 
    { 
      ENABLE_LIGHT_LEFT_MSG_INDEX = 3,
      ENABLE_LIGHT_RIGHT_MSG_INDEX = 7,
      LIGHT_CAN_MSG_INDEX = 4,
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
       * @return jimmbot_msgs::CanFrame 
       */
      jimmbot_msgs::CanFrame getSpeedInCan(void);

      /**
       * @brief Get the Status object
       * 
       * @return status_t 
       */
      status_t getStatus(void);

      /**
       * @brief Get the Lights In Can object
       * 
       * @param lights 
       * @return jimmbot_msgs::CanFrame 
       */
      static jimmbot_msgs::CanFrame getLightsInCan(const std::pair<bool, bool> &lights);

      /**
       * @brief 
       * 
       * @param status_frame 
       */
      void updateStatusFrame(jimmbot_msgs::CanFrame status_frame);

    private:
      // Pack a motor status struct into a compressed CAN frame
      jimmbot_msgs::CanFrame PackCompressedMotorStatus(const status_t& motorStatus);

      // Unpack a motor status struct from a compressed CAN frame
      status_t UnpackCompressedMotorStatus(const jimmbot_msgs::CanFrame& canFrame);

      /**
       * @brief 
       * 
       * @param speed 
       * @return true 
       * @return false 
       */
      bool negativeBit(const double &speed) const;

      /**
       * @brief 
       * 
       * @param speed 
       * @return double 
       */
      double linearToAngular(const double &speed) const;

      /**
       * @brief 
       * 
       * @param speed 
       * @return double 
       */
      double angularToLinear(const double &speed) const;

      /**
       * @brief Get the Wheel Diameter object
       * 
       * @return const double 
       */
      inline const double getWheelDiameter(void) const
      {
        return this->_wheel_diameter;
      }

      /**
       * @brief Get the Max Speed object
       * 
       * @return const double 
       */
      inline const double getMaxSpeed(void) const
      {
        return this->_max_speed;
      }

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
      jimmbot_msgs::CanFrame _status_frame;
      double _speed;
      uint8_t _command;
      const double _wheel_diameter = {0.1651};
      const double _max_speed = {4.0};
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
    jimmbot_msgs::CanFrame _status_frame;

    CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command, double value) : _can_wrapper(can_wrapper), 
                                                                                      _command(command),
                                                                                      _value(value) { }

    CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command, jimmbot_msgs::CanFrame status_frame) : _can_wrapper(can_wrapper), 
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