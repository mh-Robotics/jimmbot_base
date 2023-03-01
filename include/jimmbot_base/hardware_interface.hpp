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
#ifndef JIMMBOT_HARDWARE_INTERFACE_H_
#define JIMMBOT_HARDWARE_INTERFACE_H_

#include <hardware_interface/joint_command_interface.h>  // for hardware_interface::VelocityJointInterface
#include <hardware_interface/joint_state_interface.h>  // for hardware_interface::JointStateInterface
#include <hardware_interface/robot_hw.h>   // for hardware_interface::RobotHW
#include <jimmbot_msgs/CanFrameStamped.h>  // for jimmbot_msgs::CanFrameStamped
#include <jimmbot_msgs/ExtnDataStamped.h>  // for jimmbot_msgs::ExtnDataStamped
#include <ros/node_handle.h>               // for ros::NodeHandle
#include <ros/publisher.h>                 // for ros::Publisher
#include <ros/subscriber.h>                // for ros::Subscriber
#include <std_msgs/Float64.h>              // for std_msgs::Float64

// #include <set>
#include <utility>  // for std::distance

#include "can_msg_wrapper.hpp"       // for CanMsgWrapper
#include "jimmbot_base/constants.h"  // for jimmbot_base::k*

/**
 * @brief jimmbot_base namespace
 *
 */
namespace jimmbot_base {
/**
 * @brief Get the Index object
 *
 * @tparam Collection
 * @param collection
 * @param offset
 * @return auto
 */
template <typename Collection>
auto getIndex(Collection const& collection, size_t offset = 0) {
  return [&collection, offset](auto const& iterator) {
    return offset + std::distance(std::begin(collection), iterator);
  };
}

/**
 * @brief Definition of JimmBotHardwareInterface class
 *
 */
class JimmBotHardwareInterface : public hardware_interface::RobotHW {
 public:
  /**
   * @brief Joint elements structure. Used to save the data for joint
   *
   */
  typedef struct JointElements {
    double velocity_command = 0;
    double position = 0;
    double velocity = 0;
    double effort = 0;
  } joint_elemets_t;

  /**
   * @brief Construct a new Jimm Bot Hardware Interface object
   *
   */
  JimmBotHardwareInterface(void) = delete;

  /**
   * @brief Construct a new Jimm Bot Hardware Interface object
   *
   * @param nh
   * @param nh_param
   */
  JimmBotHardwareInterface(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

  /**
   * @brief Register the controller interface
   *
   */
  void registerControlInterfaces();

  /**
   * @brief Read data from hardware and update the state
   *
   */
  void readFromHardware(void);

  /**
   * @brief Write data to the hardware
   *
   */
  void writeToHardware(void);

  /**
   * @brief Get the Time Now object
   *
   * @return ros::Time
   */
  inline ros::Time getTimeNow(void) { return ros::Time::now(); }

  /**
   * @brief Get the Control Frequency object
   *
   * @return double
   */
  inline double getControlFrequency(void) { return this->control_frequency_; }

  /**
   * @brief Get the Elapsed Time object
   *
   * @param last_time
   * @return ros::Duration
   */
  inline ros::Duration getElapsedTime(ros::Time last_time) {
    return ros::Duration(this->getTimeNow() - last_time);
  }

  /**
   * @brief Callback for the received
   *
   * @param feedback_msg_array
   */
  void canFeedbackMsgCallback(
      const jimmbot_msgs::CanFrameStamped::ConstPtr& feedback_msg_array);

  /**
   * @brief
   *
   * @param extn_data_msg
   */
  void extnDataMsgCallback(
      const jimmbot_msgs::ExtnDataStamped::ConstPtr& extn_data_msg);

  /**
   * @brief
   *
   * @param extn_data_msg
   */
  void cameraTiltFrontCallback(const std_msgs::Float64::ConstPtr& angle);

  /**
   * @brief
   *
   * @param extn_data_msg
   */
  void cameraTiltBackCallback(const std_msgs::Float64::ConstPtr& angle);

 private:
  /**
   * @brief
   *
   */
  void updateJointsFromHardware(void);

  /**
   * @brief
   *
   */
  void updateSpeedToHardware(void);

  /**
   * @brief
   *
   */
  void updateAngleToKinectCameras(void);

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;

  std::vector<joint_elemets_t> joint_elements_;

  CanMsgWrapper front_left_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelFrontLeft),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelFrontLeft));
  CanMsgWrapper front_right_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelFrontRight),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelFrontRight));
  CanMsgWrapper back_left_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelBackLeft),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelBackLeft));
  CanMsgWrapper back_right_ = CanMsgWrapper(
      static_cast<uint8_t>(CanMsgWrapper::CanId::kCommandWheelBackRight),
      static_cast<uint8_t>(CanMsgWrapper::CanId::kFeedbackWheelBackRight));

  ros::NodeHandle nh_;
  ros::Publisher esp32_can_pub_;
  ros::Subscriber esp32_can_sub_;
  ros::Subscriber extn_data_sub_;
  std::pair<ros::Subscriber, ros::Subscriber> camera_tilt_sub_;
  std::pair<bool, bool> lights_;
  std::pair<float, float> camera_angles_;

  double control_frequency_;
  double max_wheel_speed_;
  std::string frame_id_;

  std::string left_wheel_front_;
  std::string left_wheel_back_;
  std::string right_wheel_front_;
  std::string right_wheel_back_;
};
}  // end namespace jimmbot_base
#endif  // end JIMMBOT_HARDWARE_INTERFACE_H_