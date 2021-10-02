#include "jimmbot_base/hil_wheel_commander.hpp"

namespace jimmbot_controller
{
  constexpr uint8_t CAN_MSG_COUNT = 4;

  HilWheelCommander::HilWheelCommander()
  {
    esp32_can_sub_ = node_.subscribe<sensor_msgs::JointState> (SUBSCRIBER_COMMAND_TOPIC, 1, &HilWheelCommander::esp32NodeCallback, this);
    extn_data_sub_ = node_.subscribe<jimmbot_msgs::extn_data> (SUBSCRIBER_EXTN_DATA_TOPIC, 1, &HilWheelCommander::extnDataMsgCallback, this);
    esp32_can_pub_ = node_.advertise<jimmbot_msgs::canFrameArray> (PUBLISHER_COMMAND_TOPIC_CAN_MSG_ARRAY, 1, false);
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
    jimmbot_msgs::canFrameArray _data_frame_array;

    _data_frame_array.header.stamp = ros::Time::now();
    _data_frame_array.header.frame_id = "/jimmbot/hw/hil/cmd";

    for(int index = 0; index < CAN_MSG_COUNT; index++)
    {
      _data_frame_array.can_frames[index].data = {0, 0, 0, 0, 0, 0, negativeBit(state->velocity[index + 2]), regulateSpeedToUInt8(state->velocity[index + 2])};
      _data_frame_array.can_frames[index].id = index;
      _data_frame_array.can_frames[index].dlc = 8;
    }

    {
      _data_frame_array.can_frames[CAN_MSG_COUNT].data = {0, 0, 0, this->lights_.first, 0, 0, 0, this->lights_.second};
      _data_frame_array.can_frames[CAN_MSG_COUNT].id = LIGHT_MSG_ID;
      _data_frame_array.can_frames[CAN_MSG_COUNT].dlc = 8;
    }

    esp32_can_pub_.publish(_data_frame_array);
  }

  void HilWheelCommander::extnDataMsgCallback(const jimmbot_msgs::extn_data::ConstPtr &extn_data_msg)
  {
    this->lights_ = {extn_data_msg->left_light_bulb, extn_data_msg->right_light_bulb};
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