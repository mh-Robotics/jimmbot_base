#include "jimmbot_base/extended_joy.hpp"

namespace jimmbot_base
{
  /**
   * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
   * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoyExtended
   * directly into base nodes.
   */
  struct TeleopTwistJoyExtended::Impl
  {
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void sendTiltAngleMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
    void sendExtnDataMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
    bool returnSwitchStateFromPush(bool state);

    ros::Time _time_old = ros::Time::now();
    bool inverse_state = false;
    int inverse_movement = 3;
    int left_light_bulb = 5;
    int right_light_bulb = 4;
    int horn = 0;

    ros::Subscriber joy_sub;
    ros::Publisher camera_possition_pub_front;
    ros::Publisher camera_possition_pub_back;
    ros::Publisher extn_data_pub_;

    std::map<std::string, int> axis_linear_map;
    std::map< std::string, std::map<std::string, double> > scale_linear_map;
  };

  /**
   * Constructs TeleopTwistJoyExtended.
   * \param nh NodeHandle to use for setting up the publisher and subscriber.
   * \param nh_param NodeHandle to use for searching for configuration parameters.
   */
  TeleopTwistJoyExtended::TeleopTwistJoyExtended(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
  {
    pimpl_ = new Impl;

    pimpl_->camera_possition_pub_front = nh->advertise<std_msgs::Float64>("camera_tilt_front", 1, true);
    pimpl_->camera_possition_pub_back = nh->advertise<std_msgs::Float64>("camera_tilt_back", 1, true);
    pimpl_->extn_data_pub_ = nh->advertise<jimmbot_msgs::ExtnDataStamped>("extn_data", 1, true);
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoyExtended::Impl::joyCallback, pimpl_);

    if (nh_param->getParam("axis_linear_tilt", pimpl_->axis_linear_map))
    {
      nh_param->getParam("max_angle_tilt", pimpl_->scale_linear_map["normal"]);
    }
    else
    {
      nh_param->param<int>("axis_linear_tilt", pimpl_->axis_linear_map["data"], 3);
      nh_param->param<double>("max_angle_tilt", pimpl_->scale_linear_map["normal"]["data"], 30);
    }

    for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
        it != pimpl_->axis_linear_map.end(); ++it)
    {
      ROS_INFO_NAMED("TeleopTwistJoyExtended", "Linear axis %s on %i at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    }

    if(!nh_param->getParam("inverse_movement", pimpl_->inverse_movement))
    {
      nh_param->param<int>("inverse_movement", pimpl_->inverse_movement, 3);
    }

    if(!nh_param->getParam("left_light_bulb", pimpl_->left_light_bulb))
    {
      nh_param->param<int>("left_light_bulb", pimpl_->left_light_bulb, 5);
    }

    if(!nh_param->getParam("right_light_bulb", pimpl_->right_light_bulb))
    {
      nh_param->param<int>("right_light_bulb", pimpl_->right_light_bulb, 4);
    }

    if(!nh_param->getParam("horn", pimpl_->horn))
    {
      nh_param->param<int>("horn", pimpl_->horn, 0);
    }
  }

  double degreeToRadian(double degree)
  {
    return (degree * M_PI / 180);
  }

  double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
                const std::map<std::string, double>& scale_map, const std::string& fieldname)
  {
    if (axis_map.find(fieldname) == axis_map.end() ||
        scale_map.find(fieldname) == scale_map.end() ||
        joy_msg->axes.size() <= axis_map.at(fieldname))
    {
      return 0.0;
    }

    return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
  }

  void TeleopTwistJoyExtended::Impl::sendTiltAngleMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                                      const std::string& which_map)
  {
    // Initializes with zeros by default.
    std_msgs::Float64 camera_front_possition_msg;
    std_msgs::Float64 camera_back_possition_msg;

    camera_front_possition_msg.data = degreeToRadian(getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "data") * -1);
    camera_back_possition_msg.data = degreeToRadian(getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "data"));

    camera_possition_pub_front.publish(camera_front_possition_msg);
    camera_possition_pub_back.publish(camera_back_possition_msg);
  }

  bool TeleopTwistJoyExtended::Impl::returnSwitchStateFromPush(bool state)
  {
    if(ros::Time::now().toSec() - _time_old.toSec() < ros::Duration(0.2).toSec())
    {
      return inverse_state;
    }
    else
    {
      if(state == true) 
      {
        if(inverse_state == false)
        {
          inverse_state = true;
        }
        else if(inverse_state == true)
        {
          inverse_state = false;
        }
      }

      _time_old = ros::Time::now();
    }

    return inverse_state;
  }

  void TeleopTwistJoyExtended::Impl::sendExtnDataMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                                     const std::string& which_map)
  {
    jimmbot_msgs::ExtnDataStamped extn_data_msg;

    extn_data_msg.header.frame_id = "/jimmbot/extn_data";
    extn_data_msg.header.stamp = ros::Time::now();
    extn_data_msg.extn_data.inverse_movement = returnSwitchStateFromPush(joy_msg->buttons[inverse_movement]);
    extn_data_msg.extn_data.left_light_bulb = (joy_msg->axes[left_light_bulb] == -1 ? true : false);
    extn_data_msg.extn_data.right_light_bulb = (joy_msg->axes[right_light_bulb] == -1 ? true : false);
    extn_data_msg.extn_data.horn = joy_msg->buttons[horn];

    extn_data_pub_.publish(extn_data_msg);
  }

  void TeleopTwistJoyExtended::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    if(joy_msg->buttons.size() > 0)
    {
      sendTiltAngleMsg(joy_msg, "normal");
      sendExtnDataMsg(joy_msg, "normal");
    }
  }
}//end namespace jimmbot_base

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extended_joy_node");

  ros::NodeHandle nh(""), nh_param("~");
  jimmbot_base::TeleopTwistJoyExtended joy_teleop_extended(&nh, &nh_param);

  ros::spin();

  return EXIT_SUCCESS;
}