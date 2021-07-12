#include "jimmbot_base/hardware_interface.hpp"
#include "controller_manager/controller_manager.h"

#include <ros/callback_queue.h>

namespace jimmbot_base
{
  JimmBotHardwareInterface::JimmBotHardwareInterface(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
  {
    if(!nh_param->getParam("control_frequency", this->control_frequency_))
    {
      nh_param->param<double>("control_frequency", this->control_frequency_, 10.0);
    }

    if(!nh_param->getParam("max_wheel_speed", this->control_frequency_))
    {
      nh_param->param<double>("max_wheel_speed", this->control_frequency_, 2.5);
    }

    if(!nh_param->getParam("command_frame_id", this->frame_id_))
    {
      nh_param->param<std::string>("command_frame_id", this->frame_id_, "/jimmbot/hw/cmd");
    }

    if(!nh_param->getParam("left_wheel_front", this->left_wheel_front_))
    {
      nh_param->param<std::string>("left_wheel_front", this->left_wheel_front_, "wheel_axis_front_left_to_wheel_front_left");
    }

    if(!nh_param->getParam("left_wheel_back", this->left_wheel_back_))
    {
      nh_param->param<std::string>("left_wheel_back", this->left_wheel_back_, "wheel_axis_back_left_to_wheel_back_left");
    }

    if(!nh_param->getParam("right_wheel_front", this->right_wheel_front_))
    {
      nh_param->param<std::string>("right_wheel_front", this->right_wheel_front_, "wheel_axis_front_right_to_wheel_front_right");
    }

    if(!nh_param->getParam("right_wheel_back", this->right_wheel_back_))
    {
      nh_param->param<std::string>("right_wheel_back", this->right_wheel_back_, "wheel_axis_back_right_to_wheel_back_right");
    }

    this->registerControlInterfaces();

    this->esp32_can_sub_ = nh_.subscribe<jimmbot_msgs::canFrameArray>("feedback/can_msg_array", 1, &JimmBotHardwareInterface::canFeedbackMsgCallback, this);
    this->extn_data_sub_ = nh_.subscribe<jimmbot_msgs::extn_data>("extn_data", 1, &JimmBotHardwareInterface::extnDataMsgCallback, this);
    this->esp32_can_pub_ = nh_.advertise<jimmbot_msgs::canFrameArray>("command/can_msg_array", 1, false);
  }

  void JimmBotHardwareInterface::registerControlInterfaces()
  {
    std::set<std::string> joints = {this->left_wheel_front_, this->left_wheel_back_, 
                                    this->right_wheel_front_, this->right_wheel_back_};

    const auto index = getIndex(joints, 0);

    for (auto joint = std::begin(joints); joint != std::end(joints); ++joint)
    {
      hardware_interface::JointStateHandle joints_state_handle(*joint,
                                                               &joint_elements_[index(joint)].position, 
                                                               &joint_elements_[index(joint)].velocity, 
                                                               &joint_elements_[index(joint)].effort);
      this->joint_state_interface_.registerHandle(joints_state_handle);
      
      hardware_interface::JointHandle joints_velocity_handle(this->joint_state_interface_.getHandle(*joint), 
                                                             &joint_elements_[index(joint)].velocity_command);
      this->joint_velocity_interface_.registerHandle(joints_velocity_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&joint_velocity_interface_);
  }

  void JimmBotHardwareInterface::writeToHardware(void)
  {
    std::vector<CanMsgWrapperCommand> speed_commands
    {
      CanMsgWrapperCommand{this->front_left_, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].velocity_command},
      CanMsgWrapperCommand{this->front_right_, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].velocity_command},
      CanMsgWrapperCommand{this->back_left_, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].velocity_command},
      CanMsgWrapperCommand{this->back_right_, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].velocity_command}
    };

    for (auto& speed_cmd : speed_commands)
    {
      speed_cmd.execute();
    }

    this->updateSpeedToHardware();
  }

  void JimmBotHardwareInterface::readFromHardware(void)
  {
    std::vector<CanMsgWrapperCommand> feedbacks
    {
      CanMsgWrapperCommand{this->front_left_, CanMsgWrapperCommand::Command::MOTOR_STATUS},
      CanMsgWrapperCommand{this->front_right_, CanMsgWrapperCommand::Command::MOTOR_STATUS},
      CanMsgWrapperCommand{this->back_left_, CanMsgWrapperCommand::Command::MOTOR_STATUS},
      CanMsgWrapperCommand{this->back_right_, CanMsgWrapperCommand::Command::MOTOR_STATUS}
    };

    for (auto& feedback : feedbacks)
    {
      feedback.execute();
    }
  
    this->updateJointsFromHardware();
  }

  void JimmBotHardwareInterface::updateJointsFromHardware(void)
  {
    {
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].position = this->front_left_.getStatus()._position;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].position = this->front_right_.getStatus()._position;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].position = this->back_left_.getStatus()._position;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].position = this->back_right_.getStatus()._position;
    }

    {
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].velocity = this->front_left_.getStatus()._speed;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].velocity = this->front_right_.getStatus()._speed;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].velocity = this->back_left_.getStatus()._speed;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].velocity = this->back_right_.getStatus()._speed;
    }

    {
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].effort = this->front_left_.getStatus()._effort;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].effort = this->front_right_.getStatus()._effort;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].effort = this->back_left_.getStatus()._effort;
      joint_elements_[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].effort = this->back_right_.getStatus()._effort;
    }
  }

  void JimmBotHardwareInterface::updateSpeedToHardware(void)
  {
    jimmbot_msgs::canFrameArray _data_frame_array;

    _data_frame_array.header.stamp = ros::Time::now();
    _data_frame_array.header.frame_id = this->frame_id_;

    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)] = this->front_left_.getSpeedInCan();
    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)] = this->front_right_.getSpeedInCan();
    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)] = this->back_left_.getSpeedInCan();
    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)] = this->back_right_.getSpeedInCan();
    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::LightsId::LIGHT_CAN_MSG_INDEX)] = CanMsgWrapper::getLightsInCan(this->lights_);

    esp32_can_pub_.publish(_data_frame_array);
  }

  void JimmBotHardwareInterface::canFeedbackMsgCallback(const jimmbot_msgs::canFrameArray::ConstPtr &feedback_msg_array)
  {
    std::vector<CanMsgWrapperCommand> update_status_frames
    {
      CanMsgWrapperCommand{this->front_left_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedback_msg_array->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)]},
      CanMsgWrapperCommand{this->front_right_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedback_msg_array->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)]},
      CanMsgWrapperCommand{this->back_left_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedback_msg_array->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)]},
      CanMsgWrapperCommand{this->back_right_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedback_msg_array->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)]}
    };

    for (auto& status_frame : update_status_frames)
    {
      status_frame.execute();
    }
  }

  void JimmBotHardwareInterface::extnDataMsgCallback(const jimmbot_msgs::extn_data::ConstPtr &extn_data_msg)
  {
    this->lights_ = {extn_data_msg->left_light_bulb, extn_data_msg->right_light_bulb};
  }

  void controlLoopCallback(jimmbot_base::JimmBotHardwareInterface &jimmbot_base,
                           controller_manager::ControllerManager &controller_manager,
                           ros::Time last_time)
  {
    jimmbot_base.readFromHardware();
    controller_manager.update(jimmbot_base.getTimeNow(), jimmbot_base.getElapsedTime(last_time));
    jimmbot_base.writeToHardware();
  }

}//end namespace jimmbot_base

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hardware_interface_node");
  ros::NodeHandle nh, nh_param("~");
  const double _one_second = 1;

  jimmbot_base::JimmBotHardwareInterface hardware_interface(&nh, &nh_param);

  controller_manager::ControllerManager controller_manager(&hardware_interface);
  
  ros::CallbackQueue jimmbot_queue;
  ros::AsyncSpinner jimmbot_spinner(_one_second, &jimmbot_queue);
  
  ros::TimerOptions control_loop_timer(ros::Duration(_one_second / hardware_interface.getControlFrequency()),
                                       boost::bind(jimmbot_base::controlLoopCallback, boost::ref(hardware_interface), boost::ref(controller_manager), hardware_interface.getTimeNow()),
                                       &jimmbot_queue);

  ros::Timer control_loop_callback = nh.createTimer(control_loop_timer);

  jimmbot_spinner.start();
  ros::spin();

  return EXIT_SUCCESS;
}