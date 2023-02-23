/**
 * @file hil_wheel_commander.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ___JIMMBOT_HIL_WHEEL_COMMANDER___
#define ___JIMMBOT_HIL_WHEEL_COMMANDER___

#include <ros/ros.h>
#include <jimmbot_msgs/CanFrame.h> // for jimmbot_msg::CanFrame
#include <jimmbot_msgs/CanFrameStamped.h> // for jimmbot_msg::CanFrameStamped
#include <jimmbot_msgs/ExtnDataStamped.h>
#include <sensor_msgs/JointState.h>
#include <string>

constexpr auto kLightCanMsgId{0x31};

constexpr auto kFeedbackTopicCanMsg{"/esp32/feedback/can_msg"};
constexpr auto kCommandTopicCanMsg{"/esp32/command/can_msg"};

#define SUBSCRIBER_COMMAND_TOPIC "/joint_states"
#define SUBSCRIBER_EXTN_DATA_TOPIC "/jimmbot_controller/jimmbot_extn_data_controller/extn_data"

namespace jimmbot_controller
{
  class HilWheelCommander 
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

      HilWheelCommander();
      void esp32NodeCallback(const sensor_msgs::JointState::ConstPtr& state);
      void extnDataMsgCallback(const jimmbot_msgs::ExtnDataStamped::ConstPtr &extn_data_msg);
      void esp32NodeFeedbackCallback(const jimmbot_msgs::CanFrameStamped::ConstPtr &data_msg);

      bool negativeBit(const double &speed) const;
      uint8_t regulateSpeedToUInt8(const double &speed) const;
      double linearToAngular(const double &speed) const;
      double angularToLinear(const double &speed) const;

      inline const double getWheelDiameter(void) const
      {
        return this->_wheel_diameter;
      }

      inline const double getMaxSpeed(void) const
      {
        return this->_max_speed;
      }

    private:
      ros::NodeHandle node_;

      ros::Publisher esp32_can_pub_;
      ros::Subscriber esp32_can_sub_;
      ros::Subscriber esp32_can_feedback_sub_;
      ros::Subscriber extn_data_sub_;
      const double _wheel_diameter = {0.1651};
      const double _max_speed = {4.0};
      std::pair<bool, bool> lights_ {false, false};
  };
}//end namespace jimmbot_controller
#endif//end ___JIMMBOT_HIL_WHEEL_COMMANDER___