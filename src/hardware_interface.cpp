#include "jimmbot_base/hardware_interface.hpp"
#include "controller_manager/controller_manager.h"

#include <ros/callback_queue.h>

namespace jimmbot_base
{
  JimmBotHardwareInterface::JimmBotHardwareInterface(ros::NodeHandle* nh, ros::NodeHandle* nh_param) 
    : lights_(false, false)
    , camera_angles_(HardwareInterfaceConstants::ZERO_DOUBLE, HardwareInterfaceConstants::ZERO_DOUBLE)
    , control_frequency_(HardwareInterfaceConstants::DEFAULT_CONTROL_FREQUENCY)
    , max_wheel_speed_(HardwareInterfaceConstants::DEFAULT_MAX_WHEEL_SPEED)
  {
    if(!nh_param->getParam(HardwareInterfaceConstants::CONTROL_FREQUENCY, this->control_frequency_))
    {
      nh_param->param<double>(HardwareInterfaceConstants::CONTROL_FREQUENCY, this->control_frequency_, HardwareInterfaceConstants::DEFAULT_CONTROL_FREQUENCY);
    }

    if(!nh_param->getParam(HardwareInterfaceConstants::MAX_WHEEL_SPEED, this->max_wheel_speed_))
    {
      nh_param->param<double>(HardwareInterfaceConstants::MAX_WHEEL_SPEED, this->max_wheel_speed_, HardwareInterfaceConstants::DEFAULT_MAX_WHEEL_SPEED);
    }

    if(!nh_param->getParam(HardwareInterfaceConstants::COMMAND_FRAME_ID, this->frame_id_))
    {
      nh_param->param<std::string>(HardwareInterfaceConstants::COMMAND_FRAME_ID, this->frame_id_, HardwareInterfaceConstants::DEFAULT_COMMAND_FRAME_ID);
    }

    if(!nh_param->getParam(HardwareInterfaceConstants::LEFT_WHEEL_FRONT, this->left_wheel_front_))
    {
      nh_param->param<std::string>(HardwareInterfaceConstants::LEFT_WHEEL_FRONT, this->left_wheel_front_, HardwareInterfaceConstants::DEFAULT_LEFT_WHEEL_FRONT);
    }

    if(!nh_param->getParam(HardwareInterfaceConstants::LEFT_WHEEL_BACK, this->left_wheel_back_))
    {
      nh_param->param<std::string>(HardwareInterfaceConstants::LEFT_WHEEL_BACK, this->left_wheel_back_, HardwareInterfaceConstants::DEFAULT_LEFT_WHEEL_BACK);
    }

    if(!nh_param->getParam(HardwareInterfaceConstants::RIGHT_WHEEL_FRONT, this->right_wheel_front_))
    {
      nh_param->param<std::string>(HardwareInterfaceConstants::RIGHT_WHEEL_FRONT, this->right_wheel_front_, HardwareInterfaceConstants::DEFAULT_RIGHT_WHEEL_FRONT);
    }

    if(!nh_param->getParam(HardwareInterfaceConstants::RIGHT_WHEEL_BACK, this->right_wheel_back_))
    {
      nh_param->param<std::string>(HardwareInterfaceConstants::RIGHT_WHEEL_BACK, this->right_wheel_back_, HardwareInterfaceConstants::DEFAULT_RIGHT_WHEEL_BACK);
    }

    this->registerControlInterfaces();

    this->esp32_can_sub_ = nh_.subscribe<jimmbot_msgs::CanFrameStamped>(HardwareInterfaceConstants::DEFAULT_CAN_MSG_FEEDBACK_TOPIC, 1, &JimmBotHardwareInterface::canFeedbackMsgCallback, this);
    this->extn_data_sub_ = nh_.subscribe<jimmbot_msgs::ExtnDataStamped>(HardwareInterfaceConstants::DEFAULT_EXTENDED_DATA_TOPIC, 1, &JimmBotHardwareInterface::extnDataMsgCallback, this);
    this->camera_tilt_sub_.first = nh_.subscribe<std_msgs::Float64>(HardwareInterfaceConstants::DEFAULT_CAMERA_TILT_FRONT_TOPIC, 1, &JimmBotHardwareInterface::cameraTiltFrontCallback, this);
    this->camera_tilt_sub_.second = nh_.subscribe<std_msgs::Float64>(HardwareInterfaceConstants::DEFAULT_CAMERA_TILT_BACK_TOPIC, 1, &JimmBotHardwareInterface::cameraTiltBackCallback, this);
    this->esp32_can_pub_ = nh_.advertise<jimmbot_msgs::CanFrameStamped>(HardwareInterfaceConstants::DEFAULT_CAN_MSG_COMMAND_TOPIC, 1, false);
  }

  void JimmBotHardwareInterface::registerControlInterfaces()
  {
    std::set<std::string> joints = {this->left_wheel_front_, this->left_wheel_back_, 
                                    this->right_wheel_front_, this->right_wheel_back_};

    const auto index = getIndex(joints, HardwareInterfaceConstants::ZERO_INT);

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

    for (auto& speed_command : speed_commands)
    {
      speed_command.execute();
    }

    //@todo write to AUX kinect
    this->updateSpeedToHardware();
  }

  void JimmBotHardwareInterface::readFromHardware(void)
  {
    //Status is updated from the feedback callback, then here we read those values and update joint states
    //@todo check if we need a mutex so we don't read while the callback is updating status frame 
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
    jimmbot_msgs::CanFrameStamped _data_frame;

    {
      _data_frame.header.stamp = ros::Time::now();
      _data_frame.header.frame_id = this->frame_id_;
      _data_frame.can_frame = this->front_left_.getSpeedInCan();
      esp32_can_pub_.publish(_data_frame);
    }

    {
      _data_frame.header.stamp = ros::Time::now();
      _data_frame.header.frame_id = this->frame_id_;
      _data_frame.can_frame = this->front_right_.getSpeedInCan();
      esp32_can_pub_.publish(_data_frame);
    }

    {
      _data_frame.header.stamp = ros::Time::now();
      _data_frame.header.frame_id = this->frame_id_;
      _data_frame.can_frame = this->back_left_.getSpeedInCan();
      esp32_can_pub_.publish(_data_frame);
    }

    {
      _data_frame.header.stamp = ros::Time::now();
      _data_frame.header.frame_id = this->frame_id_;
      _data_frame.can_frame = this->back_right_.getSpeedInCan();
      esp32_can_pub_.publish(_data_frame);
    }

    {
      _data_frame.header.stamp = ros::Time::now();
      _data_frame.header.frame_id = this->frame_id_;
      _data_frame.can_frame = CanMsgWrapper::getLightsInCan(this->lights_);
      esp32_can_pub_.publish(_data_frame);
    }
    
  }

  void JimmBotHardwareInterface::updateAngleToKinectCameras(void)
  {
    std_msgs::Float64 _angle;

    _angle.data = (std::get<HardwareInterfaceConstants::FIRST>(this->camera_angles_) == 
                   std::get<HardwareInterfaceConstants::SECOND>(this->camera_angles_) ? 
                   std::get<HardwareInterfaceConstants::FIRST>(this->camera_angles_) : 
                   std::get<HardwareInterfaceConstants::SECOND>(this->camera_angles_));

    //todo publish angle to topic
  }

  void JimmBotHardwareInterface::canFeedbackMsgCallback(const jimmbot_msgs::CanFrameStamped::ConstPtr &feedback_msg)
  {
    std::vector<CanMsgWrapperCommand> update_status_frames_commands
    {
      //todo: Filter by proper ID
      CanMsgWrapperCommand{this->front_left_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{this->front_right_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{this->back_left_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{this->back_right_, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedback_msg->can_frame}
    };

    for (auto& status_frame_command : update_status_frames_commands)
    {
      status_frame_command.execute();
    }
  }

  void JimmBotHardwareInterface::extnDataMsgCallback(const jimmbot_msgs::ExtnDataStamped::ConstPtr &extn_data_msg)
  {
    this->lights_ = {extn_data_msg->extn_data.left_light_bulb, extn_data_msg->extn_data.right_light_bulb};
  }

  void JimmBotHardwareInterface::cameraTiltFrontCallback(const std_msgs::Float64::ConstPtr &angle)
  {
    this->camera_angles_.first = angle->data;
  }

  void JimmBotHardwareInterface::cameraTiltBackCallback(const std_msgs::Float64::ConstPtr &angle)
  {
    this->camera_angles_.second = angle->data;
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