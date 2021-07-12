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
#ifndef JIMMBOT_HARDWARE_INTERFACE_H
#define JIMMBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <set>

#include <jimmbot_msgs/extn_data.h>
#include "can_msg_wrapper.hpp"

/**
 * @brief 
 * 
 */
namespace jimmbot_base
{
  /**
   * @brief Get the Index object
   * 
   * @tparam Collection 
   * @param collection 
   * @param offset 
   * @return auto 
   */
  template<typename Collection>
  auto getIndex(Collection const& collection, size_t offset = 0)
  {
    return [&collection, offset](auto const& iterator)
    {
      return offset + std::distance(begin(collection), iterator);
    };
  }

  /**
   * @brief 
   * 
   */
  class JimmBotHardwareInterface : public hardware_interface::RobotHW
  {
  public:

    /**
     * @brief 
     * 
     */
    typedef struct JointElements
    {
      double velocity_command = 0;
      double position = 0;
      double velocity = 0;
      double effort = 0;
    }joint_elemets_t;

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
     * @brief 
     * 
     */
    void registerControlInterfaces();

    /**
     * @brief 
     * 
     */
    void readFromHardware(void);

    /**
     * @brief 
     * 
     */
    void writeToHardware(void);

    /**
     * @brief Get the Time Now object
     * 
     * @return ros::Time 
     */
    inline ros::Time getTimeNow(void)
    {
      return ros::Time::now();
    }

    /**
     * @brief Get the Control Frequency object
     * 
     * @return double 
     */
    inline double getControlFrequency(void)
    {
      return this->control_frequency_;
    }

    /**
     * @brief Get the Elapsed Time object
     * 
     * @param last_time 
     * @return ros::Duration 
     */
    inline ros::Duration getElapsedTime(ros::Time last_time)
    {
      return ros::Duration(this->getTimeNow() - last_time);
    }

    /**
     * @brief 
     * 
     * @param feedback_msg_array 
     */
    void canFeedbackMsgCallback(const jimmbot_msgs::canFrameArray::ConstPtr &feedback_msg_array);

    /**
     * @brief 
     * 
     * @param extn_data_msg 
     */
    void extnDataMsgCallback(const jimmbot_msgs::extn_data::ConstPtr &extn_data_msg);

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

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface joint_velocity_interface_;

    // std::vector<joint_elemets_t> joint_elements_;
    joint_elemets_t joint_elements_[4];

    CanMsgWrapper front_left_ = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_FRONT_LEFT);
    CanMsgWrapper front_right_ = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_FRONT_RIGHT);
    CanMsgWrapper back_left_ = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_BACK_LEFT);
    CanMsgWrapper back_right_ = CanMsgWrapper(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT, CanMsgWrapper::CanId::FEEDBACK_WHEEL_BACK_RIGHT);

    ros::NodeHandle nh_;
    ros::Publisher esp32_can_pub_;
    ros::Subscriber esp32_can_sub_;
    ros::Subscriber extn_data_sub_;
    std::pair<bool, bool> lights_ {false, false};

    double control_frequency_;
    double max_wheel_speed_;
    std::string frame_id_;

    std::string left_wheel_front_;
    std::string left_wheel_back_;
    std::string right_wheel_front_;
    std::string right_wheel_back_;

  };
}//end namespace jimmbot_base
#endif//end JIMMBOT_HARDWARE_INTERFACE_H