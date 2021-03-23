#include "jimmbot_base/hil_wheel_commander.hpp"

namespace jimmbot_controller
{
  constexpr uint8_t CAN_MSG_COUNT = 4;

  HilWheelCommander::HilWheelCommander()
  {
    esp32_can_sub_ = node_.subscribe<sensor_msgs::JointState> (SUBSCRIBER_COMMAND_TOPIC, 1, &HilWheelCommander::esp32NodeCallback, this);
    esp32_can_pub_ = node_.advertise<jimmbot_msgs::canFrameArray> (PUBLISHER_COMMAND_TOPIC_CAN_MSG_ARRAY, 1, false);
  }

  bool negativeBit(double speed)
  {
    return std::round(speed) < 0 ? true : false;
  }

  uint8_t regulateSpeedToUInt8(double speed)
  {
    if(speed >= -1 && speed <= 1)
    {
      return 0;
    }

    return negativeBit(std::round(speed)) ? std::round(speed * -255 / 25) : std::round(speed * 255 / 25);
  }

  void HilWheelCommander::esp32NodeCallback(const sensor_msgs::JointState::ConstPtr& state)
  {
    jimmbot_msgs::canFrameArray _data_frame_array;

    _data_frame_array.header.stamp = ros::Time::now();
    _data_frame_array.header.frame_id = "can_testing_node";

    for(int index = 0; index < CAN_MSG_COUNT; index++)
    {
      _data_frame_array.can_frames[index].header.stamp = ros::Time::now();
      _data_frame_array.can_frames[index].header.seq = _data_frame_array.header.seq;
      _data_frame_array.can_frames[index].header.frame_id = "can_testing_node";
      _data_frame_array.can_frames[index].data = {0, 0, 0, 0, 0, 0, negativeBit(state->velocity[index]), regulateSpeedToUInt8(state->velocity[index])};
      _data_frame_array.can_frames[index].id = index;
      _data_frame_array.can_frames[index].dlc = 8;
    }

    {
      _data_frame_array.can_frames[CAN_MSG_COUNT].header.stamp = ros::Time::now();
      _data_frame_array.can_frames[CAN_MSG_COUNT].header.seq = _data_frame_array.header.seq;
      _data_frame_array.can_frames[CAN_MSG_COUNT].header.frame_id = "can_testing_node";
      _data_frame_array.can_frames[CAN_MSG_COUNT].data = {0, 0, 0, 1, 0, 0, 0, 1};
      _data_frame_array.can_frames[CAN_MSG_COUNT].id = LIGHT_MSG_ID;
      _data_frame_array.can_frames[CAN_MSG_COUNT].dlc = 8;
    }

    esp32_can_pub_.publish(_data_frame_array);
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