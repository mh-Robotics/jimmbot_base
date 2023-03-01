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
#ifndef JIMMBOT_BASE_CAN_MSG_WRAPPER_H_
#define JIMMBOT_BASE_CAN_MSG_WRAPPER_H_

#include <jimmbot_msgs/CanFrame.h>         // for jimmbot_msg::CanFrame
#include <jimmbot_msgs/CanFrameStamped.h>  // for jimmbot_msg::CanFrameStamped

#include <unordered_map>  // for std::unordered_map

#include "can_packt.h"  // for CanPackt

namespace jimmbot_base {
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

std::ostream& operator<<(std::ostream& os, const wheel_status_t& obj) {
  os << "Command: " << obj.command_id << ", Effort: " << obj.effort
     << ", Position: " << obj.position << ", RPM: " << obj.rpm
     << ", Velocity: " << obj.velocity << std::endl;

  return os;
}

class CanMsgWrapper {
  /**
   * @brief An enum representing the different CAN IDs
   */
  // todo remove public, and find a way to set these
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
   * @return wheel_status_t
   */
  wheel_status_t getStatus(void);

  /**
   * @brief Get the Lights In Can object
   *
   * @param lights
   * @return jimmbot_msgs::CanFrame
   */
  jimmbot_msgs::CanFrame getLightsInCan(const std::pair<bool, bool>& lights);

  /**
   * @brief
   *
   * @param status_frame
   */
  void updateStatusFrame(jimmbot_msgs::CanFrame status_frame);

  inline uint8_t TransmitId() { return canpressor_->TransmitId(); };
  inline uint8_t ReceiveId() { return canpressor_->ReceiveId(); };

 private:
  /**
   * @brief
   *
   * @param speed
   * @return true
   * @return false
   */
  bool negativeBit(const double& speed) const;

  /**
   * @brief
   *
   * @param speed
   * @return double
   */
  double linearToAngular(const double& speed) const;

  /**
   * @brief
   *
   * @param speed
   * @return double
   */
  double angularToLinear(const double& speed) const;

  /**
   * @brief Get the Wheel Diameter object
   *
   * @return const double
   */
  inline const double getWheelDiameter(void) const { return wheel_diameter_; }

  /**
   * @brief Get the Max Speed object
   *
   * @return const double
   */
  inline const double getMaxSpeed(void) const { return max_speed_; }

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

  std::unique_ptr<CanPackt> canpressor_;
  jimmbot_msgs::CanFrame status_frame_;
  double speed_;
  uint8_t command_;
  static constexpr double wheel_diameter_{0.1651};
  static constexpr double max_speed_{4.0};
};

class Command {
  virtual void execute() = 0;
};

class CanMsgWrapperCommand : Command {
  /**
   * @brief An enum representing the different commands
   */
  // todo remove public, and find a way to set these
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
  CanMsgWrapper& can_wrapper_;
  Command command_;
  double speed_{};
  jimmbot_msgs::CanFrame status_frame_;

  CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command,
                       double value)
      : can_wrapper_(can_wrapper), command_(command), speed_(value) {}

  CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command,
                       jimmbot_msgs::CanFrame status_frame)
      : can_wrapper_(can_wrapper),
        command_(command),
        status_frame_(status_frame) {}

  CanMsgWrapperCommand(CanMsgWrapper& can_wrapper, Command command)
      : can_wrapper_(can_wrapper), command_(command) {}

  std::unordered_map<Command, std::function<void()>> type2func{
      {Command::kWheelOff,
       [&]() {
         return can_wrapper_.setSpeed(
             static_cast<uint8_t>(Command::kWheelSpeed), 0.0);
       }},
      {Command::kWheelOn,
       [&]() {
         return can_wrapper_.setSpeed(
             static_cast<uint8_t>(Command::kWheelSpeed), 0.0);
       }},
      {Command::kWheelStatus, [&]() { return can_wrapper_.getStatus(); }},
      {Command::kWheelStatusUpdate,
       [&]() { return can_wrapper_.updateStatusFrame(status_frame_); }},
      {Command::kWheelSpeed, [&]() {
         return can_wrapper_.setSpeed(
             static_cast<uint8_t>(Command::kWheelSpeed), speed_);
       }}};

  void execute() override;
};

}  // end namespace jimmbot_base
#endif  // end JIMMBOT_BASE_CAN_MSG_WRAPPER_H_