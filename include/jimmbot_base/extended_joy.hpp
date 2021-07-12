/**
 * @file extended_joy.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-05-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ___JIMMBOT_TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_EXTENDED___
#define ___JIMMBOT_TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_EXTENDED___

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"

#include <jimmbot_msgs/extn_data.h>

#include <map>
#include <string>

namespace jimmbot_base
{
  class TeleopTwistJoyExtended
  {
  public:
    TeleopTwistJoyExtended(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

  private:
    struct Impl;
    Impl* pimpl_;
  };
}//end namespace jimmbot_base
#endif//end ___JIMMBOT_TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_EXTENDED___