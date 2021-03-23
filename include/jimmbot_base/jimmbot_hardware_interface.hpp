/**
 * @file jimmbot_hardware_interface.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-03-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ___JIMMBOT_HARDWARE_INTERFACE___
#define ___JIMMBOT_HARDWARE_INTERFACE___

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <set>

#include "can_msg_wrapper.hpp"

namespace jimmbot_base
{
  template<typename Collection>
  auto getIndex(Collection const& collection, size_t offset = 0)
  {
    return [&collection, offset](auto const& iterator)
    {
      return offset + std::distance(begin(collection), iterator);
    };
  }

  class JimmBotHardwareInterface : public hardware_interface::RobotHW
  {
  public:

    typedef struct JointElements
    {
      double velocity_command = 0;
      double position = 0;
      double velocity = 0;
      double effort = 0;
    }joint_elemets_t;

    JimmBotHardwareInterface(void);

    void registerControlInterfaces();
    void readFromHardware(void);
    void writeToHardware(void);

    inline ros::Time getTimeNow(void)
    {
      return ros::Time::now();
    }

    inline ros::Duration getElapsedTime(ros::Time last_time)
    {
      return ros::Duration(this->getTimeNow() - last_time);
    }

    void canFeedbackMsgCallback(const jimmbot_msgs::canFrameArray::ConstPtr &feedbackMsgArray);

  private:
    void updateJointsFromHardware(void);
    void updateSpeedToHardware(void);

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::VelocityJointInterface _joint_velocity_interface;

    // std::vector<joint_elemets_t> _joint_elements;
    joint_elemets_t _joint_elements[4];

    CanMsgWrapper _front_left = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_FRONT_LEFT);
    CanMsgWrapper _front_right = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_FRONT_RIGHT);
    CanMsgWrapper _back_left = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_BACK_LEFT);
    CanMsgWrapper _back_right = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_BACK_RIGHT);

    ros::NodeHandle node_;
    ros::Publisher esp32_can_pub_;
    ros::Subscriber esp32_can_sub_;
  };
}//end namespace jimmbot_base
#endif//end ___JIMMBOT_HARDWARE_INTERFACE___