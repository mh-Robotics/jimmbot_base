/**
 * @file jimmbot_hardware_interface.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file contains the implementation of the JimmBotHardwareInterface
 * class. The JimmBotHardwareInterface is a class that implements the ROS
 * RobotHW interface. It provides an interface to interact with the hardware of
 * the JimmBot robot.
 * @version 0.1
 * @date 2021-03-23
 *
 * @copyright Copyright (c) 2020-2023, mhRobotics, Inc. All rights reserved.
 * @license This project is released under the BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "jimmbot_base/hardware_interface.hpp"  // for JimmBotHardwareInterface

#include <ros/callback_queue.h>  // for ros::CallbackQueue
#include <ros/console.h>         // for ROS_DEBUG_NAMED

#include <chrono>  // for std::chrono::milliseconds
#include <thread>  // for std::thread

#include "controller_manager/controller_manager.h"  // for controller_manager::ControllerManager

namespace {
constexpr auto kOneSecond = 1;
}  // namespace

namespace jimmbot_base {
JimmBotHardwareInterface::JimmBotHardwareInterface(
    std::reference_wrapper<ros::NodeHandle> nh,
    std::reference_wrapper<ros::NodeHandle> nh_param)
    : lights_(false, false),
      camera_angles_({}, {}),
      control_frequency_(kDefaultControlFrequency),
      max_wheel_speed_(kDefaultMaxAllowedWheelSpeed) {
  if (!nh_param.get().getParam(kControlFrequencyKey, control_frequency_)) {
    nh_param.get().param<double>(kControlFrequencyKey, control_frequency_,
                                 kDefaultControlFrequency);
  }

  if (!nh_param.get().getParam(kMaxWheelSpeedKey, max_wheel_speed_)) {
    nh_param.get().param<double>(kMaxWheelSpeedKey, max_wheel_speed_,
                                 kDefaultMaxAllowedWheelSpeed);
  }

  if (!nh_param.get().getParam(kCommandFrameIdKey, frame_id_)) {
    nh_param.get().param<std::string>(kCommandFrameIdKey, frame_id_,
                                      kDefaultCommandFrameId);
  }

  if (!nh_param.get().getParam(kLeftWheelFrontKey, left_wheel_front_)) {
    nh_param.get().param<std::string>(kLeftWheelFrontKey, left_wheel_front_,
                                      kDefaultLeftWheelFront);
  }

  if (!nh_param.get().getParam(kLeftWheelBackKey, left_wheel_back_)) {
    nh_param.get().param<std::string>(kLeftWheelBackKey, left_wheel_back_,
                                      kDefaultLeftWheelBack);
  }

  if (!nh_param.get().getParam(kRightWheelFrontKey, right_wheel_front_)) {
    nh_param.get().param<std::string>(kRightWheelFrontKey, right_wheel_front_,
                                      kDefaultRightWheelFront);
  }

  if (!nh_param.get().getParam(kRightWheelBackKey, right_wheel_back_)) {
    nh_param.get().param<std::string>(kRightWheelBackKey, right_wheel_back_,
                                      kDefaultRightWheelBack);
  }

  RegisterControlInterfaces();

  esp32_can_sub_ = nh_.subscribe<jimmbot_msgs::CanFrameStamped>(
      kDefaultFeedbackTopic, 1,
      &::jimmbot_base::JimmBotHardwareInterface::CanFeedbackMsgCallback, this);
  extn_data_sub_ = nh_.subscribe<jimmbot_msgs::ExtnDataStamped>(
      kDefaultExtendedDataTopic, 1,
      &::jimmbot_base::JimmBotHardwareInterface::ExtnDataMsgCallback, this);
  camera_tilt_sub_.first = nh_.subscribe<std_msgs::Float64>(
      kDefaultFrontCameraTiltTopic, 1,
      &::jimmbot_base::JimmBotHardwareInterface::CameraTiltFrontCallback, this);
  camera_tilt_sub_.second = nh_.subscribe<std_msgs::Float64>(
      kDefaultBackCameraTiltTopic, 1,
      &::jimmbot_base::JimmBotHardwareInterface::CameraTiltBackCallback, this);
  esp32_can_pub_ = nh_.advertise<jimmbot_msgs::CanFrameStamped>(
      kDefaultCommandTopic, 1, false);
}

void JimmBotHardwareInterface::RegisterControlInterfaces() {
  std::set<std::string> joints = {left_wheel_front_, left_wheel_back_,
                                  right_wheel_front_, right_wheel_back_};

  const auto index = GetIndex(joints, 0);

  for (auto joint = std::begin(joints); joint != std::end(joints); ++joint) {
    hardware_interface::JointStateHandle joints_state_handle(
        *joint, &joint_elements_[index(joint)].feedback.position,
        &joint_elements_[index(joint)].feedback.velocity,
        &joint_elements_[index(joint)].feedback.effort);
    joint_state_interface_.registerHandle(joints_state_handle);

    hardware_interface::JointHandle joints_velocity_handle(
        joint_state_interface_.getHandle(*joint),
        &joint_elements_[index(joint)].command.velocity);
    joint_velocity_interface_.registerHandle(joints_velocity_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_velocity_interface_);
}

void JimmBotHardwareInterface::write(const ros::Time& /*time*/,
                                     const ros::Duration& /*period*/) {
  std::vector<CanMsgWrapperCommand> speed_commands{
      CanMsgWrapperCommand{std::ref(front_left_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[front_left_.TransmitId()].command},
      CanMsgWrapperCommand{std::ref(front_right_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[front_right_.TransmitId()].command},
      CanMsgWrapperCommand{std::ref(back_left_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[back_left_.TransmitId()].command},
      CanMsgWrapperCommand{std::ref(back_right_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[back_right_.TransmitId()].command}};

  for (auto& speed_command : speed_commands) {
    speed_command.Execute();
  }

  // @todo(mhalimi): For debug purposes, printout the status
  // ROS_WARN_NAMED("back_right_", "C: Command: %d",
  //                joint_elements_[back_right_.TransmitId()].command.command_id);
  // ROS_WARN_NAMED("back_right_", "C: Effort: %.2f",
  //                joint_elements_[back_right_.TransmitId()].command.effort);
  // ROS_WARN_NAMED("back_right_", "C: Position: %.2f",
  //                joint_elements_[back_right_.TransmitId()].command.position);
  // ROS_WARN_NAMED("back_right_", "C: RPM: %d",
  //                joint_elements_[back_right_.TransmitId()].command.rpm);
  // ROS_WARN_NAMED("back_right_", "C: Velocity: %.2f",
  //                joint_elements_[back_right_.TransmitId()].command.velocity);

  //@todo(issues/6): Write the angle to AUX kinect
  UpdateSpeedToHardware();
}

void JimmBotHardwareInterface::read(const ros::Time& /*time*/,
                                    const ros::Duration& /*period*/) {
  std::vector<CanMsgWrapperCommand> feedbacks{
      CanMsgWrapperCommand{std::ref(front_left_),
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{std::ref(front_right_),
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{std::ref(back_left_),
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{std::ref(back_right_),
                           CanMsgWrapperCommand::Command::kWheelStatus}};

  for (auto& feedback : feedbacks) {
    feedback.Execute();
  }

  UpdateJointsFromHardware();
}

void JimmBotHardwareInterface::UpdateJointsFromHardware() const {
  {
    joint_elements_[front_left_.TransmitId()].feedback =
        front_left_.GetWheelFeedbackStatus();
    joint_elements_[front_right_.TransmitId()].feedback =
        front_right_.GetWheelFeedbackStatus();
    joint_elements_[back_left_.TransmitId()].feedback =
        back_left_.GetWheelFeedbackStatus();
    joint_elements_[back_right_.TransmitId()].feedback =
        back_right_.GetWheelFeedbackStatus();

    // @todo(mhalimi): For debug purposes, printout the status
    // ROS_WARN_NAMED(
    //     "back_right_", "F: Command: %d",
    //     joint_elements_[back_right_.TransmitId()].feedback.command_id);
    // ROS_WARN_NAMED("back_right_", "F: Effort: %.2f",
    //                joint_elements_[back_right_.TransmitId()].feedback.effort);
    // ROS_WARN_NAMED("back_right_", "F: Position: %.2f",
    //                joint_elements_[back_right_.TransmitId()].feedback.position);
    // ROS_WARN_NAMED("back_right_", "F: RPM: %d",
    //                joint_elements_[back_right_.TransmitId()].feedback.rpm);
    // ROS_WARN_NAMED("back_right_", "F: Velocity: %.2f",
    //                joint_elements_[back_right_.TransmitId()].feedback.velocity);
  }
}

void JimmBotHardwareInterface::UpdateSpeedToHardware() const {
  jimmbot_msgs::CanFrameStamped data_frame;

  {
    data_frame.header.stamp = ros::Time::now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = front_left_.GetWheelCommandStatus();
    esp32_can_pub_.publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = ros::Time::now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = front_right_.GetWheelCommandStatus();
    esp32_can_pub_.publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = ros::Time::now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = back_left_.GetWheelCommandStatus();
    esp32_can_pub_.publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = ros::Time::now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = back_right_.GetWheelCommandStatus();
    esp32_can_pub_.publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = ros::Time::now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = CanMsgWrapper::GetLightsInCan(lights_);
    esp32_can_pub_.publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void JimmBotHardwareInterface::UpdateAngleToKinectCameras() {
  std_msgs::Float64 angle;

  angle.data =
      (std::get<kFirst>(camera_angles_) == std::get<kSecond>(camera_angles_)
           ? std::get<kFirst>(camera_angles_)
           : std::get<kSecond>(camera_angles_));

  // @todo(issues/6): Publish angle to Kinect AUX node topic
}

void JimmBotHardwareInterface::CanFeedbackMsgCallback(
    const jimmbot_msgs::CanFrameStamped::ConstPtr& feedback_msg) {
  std::vector<CanMsgWrapperCommand> update_status_frames_commands{
      CanMsgWrapperCommand{std::ref(front_left_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{std::ref(front_right_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{std::ref(back_left_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{std::ref(back_right_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame}};

  for (auto& status_frame_command : update_status_frames_commands) {
    status_frame_command.Execute();
  }
}

void JimmBotHardwareInterface::ExtnDataMsgCallback(
    const jimmbot_msgs::ExtnDataStamped::ConstPtr& extn_data_msg) {
  lights_ = {extn_data_msg->extn_data.left_light_bulb,
             extn_data_msg->extn_data.right_light_bulb};
}

void JimmBotHardwareInterface::CameraTiltFrontCallback(
    const std_msgs::Float64::ConstPtr& angle) {
  camera_angles_.first = angle->data;
}

void JimmBotHardwareInterface::CameraTiltBackCallback(
    const std_msgs::Float64::ConstPtr& angle) {
  camera_angles_.second = angle->data;
}

void ControlLoopCallback(
    std::reference_wrapper<jimmbot_base::JimmBotHardwareInterface> jimmbot_base,
    std::reference_wrapper<controller_manager::ControllerManager>
        controller_manager,
    ros::Time last_time) {
  jimmbot_base.get().read(ros::Time{}, ros::Duration{});

  controller_manager.get().update(jimmbot_base.get().GetTimeNow(),
                                  jimmbot_base.get().GetElapsedTime(last_time));

  jimmbot_base.get().write(ros::Time{}, ros::Duration{});
}

}  // end namespace jimmbot_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "hardware_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");

  jimmbot_base::JimmBotHardwareInterface hardware_interface(std::ref(nh),
                                                            std::ref(nh_param));

  controller_manager::ControllerManager controller_manager(&hardware_interface);

  ros::CallbackQueue jimmbot_queue;
  ros::AsyncSpinner jimmbot_spinner(kOneSecond, &jimmbot_queue);

  ros::TimerOptions control_loop_timer(
      ros::Duration(kOneSecond / hardware_interface.GetControlFrequency()),
      std::bind(jimmbot_base::ControlLoopCallback, std::ref(hardware_interface),
                std::ref(controller_manager), hardware_interface.GetTimeNow()),
      &jimmbot_queue);

  ros::Timer control_loop_callback = nh.createTimer(control_loop_timer);

  jimmbot_spinner.start();
  ros::spin();

  return EXIT_SUCCESS;
}
