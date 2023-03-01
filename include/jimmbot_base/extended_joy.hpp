/**
 * @file extended_joy.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Class for the extended JOY node. Added more custom features for custom
 * robot jimmBOT
 * @version 0.1
 * @date 2021-05-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef JIMMBOT_TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_EXTENDED_H
#define JIMMBOT_TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_EXTENDED_H

#include <jimmbot_msgs/ExtnData.h>
#include <jimmbot_msgs/ExtnDataStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

#include <map>
#include <string>

/**
 * @brief namespace jimmbot_base
 *
 */
namespace jimmbot_base {
/**
 * @brief Deffinition of the TeleopTwistJoyExtended class
 *
 */
class TeleopTwistJoyExtended {
 public:
  /**
   * @brief Construct a new Teleop Twist Joy Extended object
   *
   * @param nh Node handle
   * @param nh_param Private node handle parameters
   */
  TeleopTwistJoyExtended(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

 private:
  struct Impl;
  Impl* pimpl_;
};
}  // end namespace jimmbot_base
#endif  // end JIMMBOT_TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_EXTENDED_H
