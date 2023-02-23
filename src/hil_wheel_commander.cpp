#include "jimmbot_base/hil_wheel_commander.hpp"

#include <chrono>
#include <thread>

namespace jimmbot_controller
{
  constexpr uint8_t CAN_MSG_COUNT = 4;

  HilWheelCommander::HilWheelCommander()
  {
    esp32_can_sub_ = node_.subscribe<sensor_msgs::JointState> (SUBSCRIBER_COMMAND_TOPIC, 1, &HilWheelCommander::esp32NodeCallback, this);
    extn_data_sub_ = node_.subscribe<jimmbot_msgs::ExtnDataStamped> (SUBSCRIBER_EXTN_DATA_TOPIC, 1, &HilWheelCommander::extnDataMsgCallback, this);
    esp32_can_feedback_sub_ = node_.subscribe<jimmbot_msgs::CanFrameStamped> (kFeedbackTopicCanMsg, 1, &HilWheelCommander::esp32NodeFeedbackCallback, this);
    esp32_can_pub_ = node_.advertise<jimmbot_msgs::CanFrameStamped> (kCommandTopicCanMsg, 1, false);
  }

  bool HilWheelCommander::negativeBit(const double &speed) const
  {
    //@todo check here. Fix the firmware to have the reverse logic here. For testing, this is OK.
    return speed < 0 ? false : true;
  }

  uint8_t HilWheelCommander::regulateSpeedToUInt8(const double &speed) const
  {
    return (std::abs(this->angularToLinear(speed)) * UINT8_MAX / this->getMaxSpeed());
  }

  double HilWheelCommander::linearToAngular(const double &speed) const
  {
    return speed / this->getWheelDiameter();
  }

  double HilWheelCommander::angularToLinear(const double &speed) const
  {
    return speed * this->getWheelDiameter();
  }

  void HilWheelCommander::esp32NodeCallback(const sensor_msgs::JointState::ConstPtr& state)
  {
    jimmbot_msgs::CanFrameStamped _data_frame;

    for(int index = 0; index < CAN_MSG_COUNT; index++)
    {
      _data_frame.header.stamp = ros::Time::now();
      _data_frame.header.frame_id = "/jimmbot/hw/hil/cmd";
      _data_frame.can_frame.data = {0, 0, 0, 0, 0, 0, negativeBit(state->velocity[index + 2]), regulateSpeedToUInt8(state->velocity[index + 2])};
      _data_frame.can_frame.id = index;
      _data_frame.can_frame.dlc = 8;
      esp32_can_pub_.publish(_data_frame);
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    {
      _data_frame.header.stamp = ros::Time::now();
      _data_frame.header.frame_id = "/jimmbot/hw/hil/cmd";
      _data_frame.can_frame.data = {0, 0, 0, this->lights_.first, 0, 0, 0, this->lights_.second};
      _data_frame.can_frame.id = kLightCanMsgId;
      _data_frame.can_frame.dlc = 8;
      esp32_can_pub_.publish(_data_frame);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void HilWheelCommander::extnDataMsgCallback(const jimmbot_msgs::ExtnDataStamped::ConstPtr &extn_data_msg)
  {
    this->lights_ = {extn_data_msg->extn_data.left_light_bulb, extn_data_msg->extn_data.right_light_bulb};
  }

  double regulateSpeedToDouble(uint8_t first_byte, uint8_t second_byte)
  {
    return (((second_byte / 10) + first_byte) * 10);
  }

  double regulatePositionToDouble(uint8_t first_byte, uint8_t second_byte)
  {
    return (((second_byte / 10) + first_byte) * 10);
  }

  int regulateRpmToInt(uint8_t first_byte, uint8_t second_byte)
  {
    return (((second_byte / 10) + first_byte) * 10);
  }

  void HilWheelCommander::esp32NodeFeedbackCallback(const jimmbot_msgs::CanFrameStamped::ConstPtr &data_msg)
  {
    ROS_WARN("Id: %#x", data_msg->can_frame.id);
    ROS_WARN("Command: %d", data_msg->can_frame.data[0]);
    ROS_WARN("Effort: %d", (data_msg->can_frame.data[0] / 10));
    ROS_WARN("Position: %.2f", regulatePositionToDouble(data_msg->can_frame.data[2], data_msg->can_frame.data[3]));
    ROS_WARN("RPM: %d", regulateRpmToInt(data_msg->can_frame.data[4], data_msg->can_frame.data[5]));
    ROS_WARN("Speed: %.2f", regulateSpeedToDouble(data_msg->can_frame.data[6], data_msg->can_frame.data[7]));
  }
}//end namespace jimmbot_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_wheels_in_the_loop");
  jimmbot_controller::HilWheelCommander _hil_wheel_commander;

  ros::Rate _refresh_rate(10);
  _refresh_rate.sleep();

  ros::spin();
  return 0;
}