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
#include <std_msgs/Float64.h>
#include <utility>
#include <set>

#include <jimmbot_msgs/extn_data.h>

#include "can_msg_wrapper.hpp"

/**
 * @brief jimmbot_base namespace
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
   * @brief HardwareInterfaceConstants class with constants used in HadrwareInterface
   * 
   */
  class HardwareInterfaceConstants
  {
  protected:

    /**
     * @brief Hardware Interface Constants declared protected static inline, can be used by inheriting this class 
     * 
     */
    static inline const int FIRST{0};
    static inline const int SECOND{1};
    static inline const int ZERO_INT{0};
    static inline const double ZERO_DOUBLE{0.0};
    static inline const std::string CONTROL_FREQUENCY{"control_frequency"};
    static inline const double DEFAULT_CONTROL_FREQUENCY{10.0};
    static inline const std::string MAX_WHEEL_SPEED{"max_wheel_speed"};
    static inline const double DEFAULT_MAX_WHEEL_SPEED{2.5};
    static inline const std::string COMMAND_FRAME_ID{"command_frame_id"};
    static inline const std::string DEFAULT_COMMAND_FRAME_ID{"/jimmbot/hw/cmd"};
    static inline const std::string LEFT_WHEEL_FRONT{"left_wheel_front"};
    static inline const std::string DEFAULT_LEFT_WHEEL_FRONT{"wheel_axis_front_left_to_wheel_front_left"};
    static inline const std::string LEFT_WHEEL_BACK{"left_wheel_back"};
    static inline const std::string DEFAULT_LEFT_WHEEL_BACK{"wheel_axis_back_left_to_wheel_back_left"};
    static inline const std::string RIGHT_WHEEL_FRONT{"right_wheel_front"};
    static inline const std::string DEFAULT_RIGHT_WHEEL_FRONT{"wheel_axis_front_right_to_wheel_front_right"};
    static inline const std::string RIGHT_WHEEL_BACK{"right_wheel_back"};
    static inline const std::string DEFAULT_RIGHT_WHEEL_BACK{"wheel_axis_back_right_to_wheel_back_right"};

    static inline const std::string DEFAULT_CAN_MSG_FEEDBACK_TOPIC{"feedback/can_msg_array"};
    static inline const std::string DEFAULT_CAN_MSG_COMMAND_TOPIC{"command/can_msg_array"};
    static inline const std::string DEFAULT_EXTENDED_DATA_TOPIC{"extn_data"};
    static inline const std::string DEFAULT_CAMERA_TILT_FRONT_TOPIC{"camera_tilt_front"};
    static inline const std::string DEFAULT_CAMERA_TILT_BACK_TOPIC{"camera_tilt_front"};
  };

  /**
   * @brief Deffinition of JimmBotHardwareInterface class
   * 
   */
  class JimmBotHardwareInterface : public hardware_interface::RobotHW, private HardwareInterfaceConstants
  {
  public:

    /**
     * @brief Joint elements structure. Used to save the data for joint
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
     * @brief Callback for the received 
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

    /**
     * @brief 
     * 
     * @param extn_data_msg 
     */
    void cameraTiltFrontCallback(const std_msgs::Float64::ConstPtr &angle);

    /**
     * @brief 
     * 
     * @param extn_data_msg 
     */
    void cameraTiltBackCallback(const std_msgs::Float64::ConstPtr &angle);

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
}//end namespace jimmbot_base
#endif//end JIMMBOT_HARDWARE_INTERFACE_H