/**
 * @file hil_wheel_commander.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-08
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef ___JIMMBOT_HIL_WHEEL_COMMANDER___
#define ___JIMMBOT_HIL_WHEEL_COMMANDER___

#include <ros/ros.h>
#include <jimmbot_msgs/canFrame.h>
#include <jimmbot_msgs/canFrameArray.h>
#include <sensor_msgs/JointState.h>
#include <string>

#define LIGHT_MSG_ID 0x31

#define PUBLISHER_COMMAND_TOPIC_CAN_MSG_ARRAY "/esp32/command/can_msg_array"
#define SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY "/esp32/feedback/can_msg_array"
#define SUBSCRIBER_COMMAND_TOPIC "/joint_states"

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

    private:
      ros::NodeHandle node_;

      ros::Publisher esp32_can_pub_;
      ros::Subscriber esp32_can_sub_;
  };
}//end namespace jimmbot_controller
#endif//end ___JIMMBOT_HIL_WHEEL_COMMANDER___