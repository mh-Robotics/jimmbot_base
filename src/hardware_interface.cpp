#include "jimmbot_base/hardware_interface.hpp"  // for JimmBotHardwareInterface

#include <ros/callback_queue.h>  // for ros::CallbackQueue

#include <chrono>  // for std::chrono::milliseconds
#include <thread>  // for std::thread

#include "controller_manager/controller_manager.h"  // for controller_manager::ControllerManager

namespace jimmbot_base {
JimmBotHardwareInterface::JimmBotHardwareInterface(ros::NodeHandle* nh,
                                                   ros::NodeHandle* nh_param)
    : lights_(false, false),
      camera_angles_({}, {}),
      control_frequency_(kDefaultControlFrequency),
      max_wheel_speed_(kDefaultMaxAllowedWheelSpeed) {
  if (!nh_param->getParam(kControlFrequencyKey, control_frequency_)) {
    nh_param->param<double>(kControlFrequencyKey, control_frequency_,
                            kDefaultControlFrequency);
  }

  if (!nh_param->getParam(kMaxWheelSpeedKey, max_wheel_speed_)) {
    nh_param->param<double>(kMaxWheelSpeedKey, max_wheel_speed_,
                            kDefaultMaxAllowedWheelSpeed);
  }

  if (!nh_param->getParam(kCommandFrameIdKey, frame_id_)) {
    nh_param->param<std::string>(kCommandFrameIdKey, frame_id_,
                                 kDefaultCommandFrameId);
  }

  if (!nh_param->getParam(kLeftWheelFrontKey, left_wheel_front_)) {
    nh_param->param<std::string>(kLeftWheelFrontKey, left_wheel_front_,
                                 kDefaultLeftWheelFront);
  }

  if (!nh_param->getParam(kLeftWheelBackKey, left_wheel_back_)) {
    nh_param->param<std::string>(kLeftWheelBackKey, left_wheel_back_,
                                 kDefaultLeftWheelBack);
  }

  if (!nh_param->getParam(kRightWheelFrontKey, right_wheel_front_)) {
    nh_param->param<std::string>(kRightWheelFrontKey, right_wheel_front_,
                                 kDefaultRightWheelFront);
  }

  if (!nh_param->getParam(kRightWheelBackKey, right_wheel_back_)) {
    nh_param->param<std::string>(kRightWheelBackKey, right_wheel_back_,
                                 kDefaultRightWheelBack);
  }

  registerControlInterfaces();

  esp32_can_sub_ = nh_.subscribe<jimmbot_msgs::CanFrameStamped>(
      kDefaultFeedbackTopic, 1,
      &JimmBotHardwareInterface::canFeedbackMsgCallback, this);
  extn_data_sub_ = nh_.subscribe<jimmbot_msgs::ExtnDataStamped>(
      kDefaultExtendedDataTopic, 1,
      &JimmBotHardwareInterface::extnDataMsgCallback, this);
  camera_tilt_sub_.first = nh_.subscribe<std_msgs::Float64>(
      kDefaultFrontCameraTiltTopic, 1,
      &JimmBotHardwareInterface::cameraTiltFrontCallback, this);
  camera_tilt_sub_.second = nh_.subscribe<std_msgs::Float64>(
      kDefaultBackCameraTiltTopic, 1,
      &JimmBotHardwareInterface::cameraTiltBackCallback, this);
  esp32_can_pub_ = nh_.advertise<jimmbot_msgs::CanFrameStamped>(
      kDefaultCommandTopic, 1, false);
}

void JimmBotHardwareInterface::registerControlInterfaces() {
  std::set<std::string> joints = {left_wheel_front_, left_wheel_back_,
                                  right_wheel_front_, right_wheel_back_};

  const auto index = getIndex(joints, 0);

  for (auto joint = std::begin(joints); joint != std::end(joints); ++joint) {
    hardware_interface::JointStateHandle joints_state_handle(
        *joint, &joint_elements_[index(joint)].position,
        &joint_elements_[index(joint)].velocity,
        &joint_elements_[index(joint)].effort);
    joint_state_interface_.registerHandle(joints_state_handle);

    hardware_interface::JointHandle jointsvelocity_handle(
        joint_state_interface_.getHandle(*joint),
        &joint_elements_[index(joint)].velocity_command);
    joint_velocity_interface_.registerHandle(jointsvelocity_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_velocity_interface_);
}

void JimmBotHardwareInterface::writeToHardware(void) {
  std::vector<CanMsgWrapperCommand> speed_commands{
      CanMsgWrapperCommand{
          front_left_, CanMsgWrapperCommand::Command::kWheelSpeed,
          joint_elements_[front_left_.TransmitId()].velocity_command},
      CanMsgWrapperCommand{
          front_right_, CanMsgWrapperCommand::Command::kWheelSpeed,
          joint_elements_[front_right_.TransmitId()].velocity_command},
      CanMsgWrapperCommand{
          back_left_, CanMsgWrapperCommand::Command::kWheelSpeed,
          joint_elements_[back_left_.TransmitId()].velocity_command},
      CanMsgWrapperCommand{
          back_right_, CanMsgWrapperCommand::Command::kWheelSpeed,
          joint_elements_[back_right_.TransmitId()].velocity_command}};

  for (auto& speed_command : speed_commands) {
    speed_command.execute();
  }

  //@todo write to AUX kinect
  updateSpeedToHardware();
}

void JimmBotHardwareInterface::readFromHardware(void) {
  // Status is updated from the feedback callback, then here we read those
  // values and update joint states
  //@todo check if we need a mutex so we don't read while the callback is
  // updating status frame
  std::vector<CanMsgWrapperCommand> feedbacks{
      CanMsgWrapperCommand{front_left_,
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{front_right_,
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{back_left_,
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{back_right_,
                           CanMsgWrapperCommand::Command::kWheelStatus}};

  for (auto& feedback : feedbacks) {
    feedback.execute();
  }

  updateJointsFromHardware();
}

void JimmBotHardwareInterface::updateJointsFromHardware(void) {
  {
    joint_elements_[front_left_.TransmitId()].position =
        front_left_.getStatus().position;
    joint_elements_[front_right_.TransmitId()].position =
        front_right_.getStatus().position;
    joint_elements_[back_left_.TransmitId()].position =
        back_left_.getStatus().position;
    joint_elements_[back_right_.TransmitId()].position =
        back_right_.getStatus().position;

    // @todo(mhalimi): For debug purposes, printout the status
    // ROS_WARN_NAMED("hardware_interface: front_left_", "Id: %#x",
    // front_left_.getStatus()._id); ROS_WARN_NAMED("hardware_interface:
    // front_left_", "Command: %d", front_left_.getStatus().command_id);
    // ROS_WARN_NAMED("hardware_interface: front_left_", "Effort: %d",
    // front_left_.getStatus().effort);
    // ROS_WARN_NAMED("hardware_interface: front_left_", "Position: %.2f",
    // front_left_.getStatus().position);
    // ROS_WARN_NAMED("hardware_interface: front_left_", "RPM: %d",
    // front_left_.getStatus().rpm); ROS_WARN_NAMED("hardware_interface:
    // front_left_", "Velocity: %.2f", front_left_.getStatus().velocity);

    // ROS_WARN_NAMED("hardware_interface: front_right_", "Command: %d",
    //                front_right_.getStatus().command_id);
    // ROS_WARN_NAMED("hardware_interface: front_right_", "Effort: %d",
    //                front_right_.getStatus().effort);
    // ROS_WARN_NAMED("hardware_interface: front_right_", "Position: %.2f",
    //                front_right_.getStatus().position);
    // ROS_WARN_NAMED("hardware_interface: front_right_", "RPM: %d",
    //                front_right_.getStatus().rpm);
    // ROS_WARN_NAMED("hardware_interface: front_right_", "Velocity: %.2f",
    //                front_right_.getStatus().velocity);

    // ROS_WARN_NAMED("hardware_interface: back_left_", "Id: %#x",
    // back_left_.getStatus()._id); ROS_WARN_NAMED("hardware_interface:
    // back_left_", "Command: %d", back_left_.getStatus().command_id);
    // ROS_WARN_NAMED("hardware_interface: back_left_", "Effort: %d",
    // back_left_.getStatus().effort);
    // ROS_WARN_NAMED("hardware_interface: back_left_", "Position: %.2f",
    // back_left_.getStatus().position);
    // ROS_WARN_NAMED("hardware_interface: back_left_", "RPM: %d",
    // back_left_.getStatus().rpm); ROS_WARN_NAMED("hardware_interface:
    // back_left_", "Velocity: %.2f", back_left_.getStatus().velocity);

    // ROS_WARN_NAMED("hardware_interface: back_right_", "Id: %#x",
    // back_right_.getStatus()._id); ROS_WARN_NAMED("hardware_interface:
    // back_right_", "Command: %d", back_right_.getStatus().command_id);
    // ROS_WARN_NAMED("hardware_interface: back_right_", "Effort: %d",
    // back_right_.getStatus().effort);
    // ROS_WARN_NAMED("hardware_interface: back_right_", "Position: %.2f",
    // back_right_.getStatus().position);
    // ROS_WARN_NAMED("hardware_interface: back_right_", "RPM: %d",
    // back_right_.getStatus().rpm); ROS_WARN_NAMED("hardware_interface:
    // back_right_", "Velocity: %.2f", back_right_.getStatus().velocity);
  }

  {
    joint_elements_[front_left_.TransmitId()].velocity =
        front_left_.getStatus().velocity;
    joint_elements_[front_right_.TransmitId()].velocity =
        front_right_.getStatus().velocity;
    joint_elements_[back_left_.TransmitId()].velocity =
        back_left_.getStatus().velocity;
    joint_elements_[back_right_.TransmitId()].velocity =
        back_right_.getStatus().velocity;
  }

  {
    joint_elements_[front_left_.TransmitId()].effort =
        front_left_.getStatus().effort;
    joint_elements_[front_right_.TransmitId()].effort =
        front_right_.getStatus().effort;
    joint_elements_[back_left_.TransmitId()].effort =
        back_left_.getStatus().effort;
    joint_elements_[back_right_.TransmitId()].effort =
        back_right_.getStatus().effort;
  }
}

void JimmBotHardwareInterface::updateSpeedToHardware(void) {
  jimmbot_msgs::CanFrameStamped _data_frame;

  {
    _data_frame.header.stamp = ros::Time::now();
    _data_frame.header.frame_id = frame_id_;
    _data_frame.can_frame = front_left_.getSpeedInCan();
    esp32_can_pub_.publish(_data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    _data_frame.header.stamp = ros::Time::now();
    _data_frame.header.frame_id = frame_id_;
    _data_frame.can_frame = front_right_.getSpeedInCan();
    esp32_can_pub_.publish(_data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    _data_frame.header.stamp = ros::Time::now();
    _data_frame.header.frame_id = frame_id_;
    _data_frame.can_frame = back_left_.getSpeedInCan();
    esp32_can_pub_.publish(_data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    _data_frame.header.stamp = ros::Time::now();
    _data_frame.header.frame_id = frame_id_;
    _data_frame.can_frame = back_right_.getSpeedInCan();
    esp32_can_pub_.publish(_data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    _data_frame.header.stamp = ros::Time::now();
    _data_frame.header.frame_id = frame_id_;
    _data_frame.can_frame = CanMsgWrapper::getLightsInCan(lights_);
    esp32_can_pub_.publish(_data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void JimmBotHardwareInterface::updateAngleToKinectCameras(void) {
  std_msgs::Float64 _angle;

  _angle.data =
      (std::get<kFirst>(camera_angles_) == std::get<kSecond>(camera_angles_)
           ? std::get<kFirst>(camera_angles_)
           : std::get<kSecond>(camera_angles_));

  // todo publish angle to topic
}

void JimmBotHardwareInterface::canFeedbackMsgCallback(
    const jimmbot_msgs::CanFrameStamped::ConstPtr& feedback_msg) {
  std::vector<CanMsgWrapperCommand> update_status_frames_commands{
      // todo: Filter by proper ID
      CanMsgWrapperCommand{front_left_,
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{front_right_,
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{back_left_,
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{back_right_,
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame}};

  for (auto& status_frame_command : update_status_frames_commands) {
    status_frame_command.execute();
  }
}

void JimmBotHardwareInterface::extnDataMsgCallback(
    const jimmbot_msgs::ExtnDataStamped::ConstPtr& extn_data_msg) {
  lights_ = {extn_data_msg->extn_data.left_light_bulb,
             extn_data_msg->extn_data.right_light_bulb};
}

void JimmBotHardwareInterface::cameraTiltFrontCallback(
    const std_msgs::Float64::ConstPtr& angle) {
  camera_angles_.first = angle->data;
}

void JimmBotHardwareInterface::cameraTiltBackCallback(
    const std_msgs::Float64::ConstPtr& angle) {
  camera_angles_.second = angle->data;
}

void controlLoopCallback(
    jimmbot_base::JimmBotHardwareInterface& jimmbot_base,
    controller_manager::ControllerManager& controller_manager,
    ros::Time last_time) {
  jimmbot_base.readFromHardware();
  controller_manager.update(jimmbot_base.getTimeNow(),
                            jimmbot_base.getElapsedTime(last_time));
  jimmbot_base.writeToHardware();
}

}  // end namespace jimmbot_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "hardware_interface_node");
  ros::NodeHandle nh, nh_param("~");
  const double _one_second = 1;

  jimmbot_base::JimmBotHardwareInterface hardware_interface(&nh, &nh_param);

  controller_manager::ControllerManager controller_manager(&hardware_interface);

  ros::CallbackQueue jimmbot_queue;
  ros::AsyncSpinner jimmbot_spinner(_one_second, &jimmbot_queue);

  ros::TimerOptions control_loop_timer(
      ros::Duration(_one_second / hardware_interface.getControlFrequency()),
      boost::bind(
          jimmbot_base::controlLoopCallback, boost::ref(hardware_interface),
          boost::ref(controller_manager), hardware_interface.getTimeNow()),
      &jimmbot_queue);

  ros::Timer control_loop_callback = nh.createTimer(control_loop_timer);

  jimmbot_spinner.start();
  ros::spin();

  return EXIT_SUCCESS;
}
