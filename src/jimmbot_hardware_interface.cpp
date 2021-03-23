#include "jimmbot_base/jimmbot_hardware_interface.hpp"
#include "controller_manager/controller_manager.h"

#include <ros/callback_queue.h>

namespace jimmbot_base
{
  constexpr char WHEEL_AXIS_FRONT_LEFT_TO_WHEEL_FRONT_LEFT[] = "wheel_axis_front_left_to_wheel_front_left";
  constexpr char WHEEL_AXIS_BACK_LEFT_TO_WHEEL_BACK_LEFT[] = "wheel_axis_back_left_to_wheel_back_left";
  constexpr char WHEEL_AXIS_FRONT_RIGHT_TO_WHEEL_FRONT_RIGHT[] = "wheel_axis_front_right_to_wheel_front_right";
  constexpr char WHEEL_AXIS_BACK_RIGHT_TO_WHEEL_BACK_RIGHT[] = "wheel_axis_back_right_to_wheel_back_right";
  constexpr char CAN_COMMANT_MSG_TOPIC[] = "/esp32/command/can_msg_array";
  constexpr char CAN_FEEDBACK_MSG_TOPIC[] = "/esp32/feedback/can_msg_array";
  
  JimmBotHardwareInterface::JimmBotHardwareInterface(void)
  {
    this->registerControlInterfaces();

    this->esp32_can_sub_ = node_.subscribe<jimmbot_msgs::canFrameArray>(CAN_FEEDBACK_MSG_TOPIC, 1, &JimmBotHardwareInterface::canFeedbackMsgCallback, this);
    this->esp32_can_pub_ = node_.advertise<jimmbot_msgs::canFrameArray>(CAN_COMMANT_MSG_TOPIC, 1, false);
  }

  void JimmBotHardwareInterface::registerControlInterfaces()
  {
    std::set<std::string> _joints = {WHEEL_AXIS_FRONT_LEFT_TO_WHEEL_FRONT_LEFT, WHEEL_AXIS_BACK_LEFT_TO_WHEEL_BACK_LEFT, 
                                     WHEEL_AXIS_FRONT_RIGHT_TO_WHEEL_FRONT_RIGHT, WHEEL_AXIS_BACK_RIGHT_TO_WHEEL_BACK_RIGHT};

    const auto index = getIndex(_joints, 0);

    for (auto joint = std::begin(_joints); joint != std::end(_joints); ++joint)
    {
      hardware_interface::JointStateHandle _joints_state_handle(*joint,
                                                                &_joint_elements[index(joint)].position, 
                                                                &_joint_elements[index(joint)].velocity, 
                                                                &_joint_elements[index(joint)].effort);
      this->_joint_state_interface.registerHandle(_joints_state_handle);
      
      hardware_interface::JointHandle _joints_velocity_handle(this->_joint_state_interface.getHandle(*joint), 
                                                              &_joint_elements[index(joint)].velocity_command);
      this->_joint_velocity_interface.registerHandle(_joints_velocity_handle);
    }

    registerInterface(&_joint_state_interface);
    registerInterface(&_joint_velocity_interface);
  }

  void JimmBotHardwareInterface::writeToHardware(void)
  {
    std::vector<CanMsgWrapperCommand> speed_commands
    {
      CanMsgWrapperCommand{this->_front_left, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].velocity_command},
      CanMsgWrapperCommand{this->_front_right, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].velocity_command},
      CanMsgWrapperCommand{this->_back_left, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].velocity_command},
      CanMsgWrapperCommand{this->_back_right, CanMsgWrapperCommand::Command::MOTOR_SPEED, 
                           _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].velocity_command}
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
      CanMsgWrapperCommand{this->_front_left, CanMsgWrapperCommand::Command::MOTOR_STATUS},
      CanMsgWrapperCommand{this->_front_right, CanMsgWrapperCommand::Command::MOTOR_STATUS},
      CanMsgWrapperCommand{this->_back_left, CanMsgWrapperCommand::Command::MOTOR_STATUS},
      CanMsgWrapperCommand{this->_back_right, CanMsgWrapperCommand::Command::MOTOR_STATUS}
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
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].position = this->_front_left.getStatus()._position;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].position = this->_front_right.getStatus()._position;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].position = this->_back_left.getStatus()._position;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].position = this->_back_right.getStatus()._position;
    }

    {
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].velocity = this->_front_left.getStatus()._speed;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].velocity = this->_front_right.getStatus()._speed;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].velocity = this->_back_left.getStatus()._speed;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].velocity = this->_back_right.getStatus()._speed;
    }

    {
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)].effort = this->_front_left.getStatus()._effort;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)].effort = this->_front_right.getStatus()._effort;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)].effort = this->_back_left.getStatus()._effort;
      _joint_elements[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)].effort = this->_back_right.getStatus()._effort;
    }
  }

  void JimmBotHardwareInterface::updateSpeedToHardware(void)
  {
    jimmbot_msgs::canFrameArray _data_frame_array;

    _data_frame_array.header.stamp = ros::Time::now();
    _data_frame_array.header.frame_id = "can_speed_updater";

    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)] = this->_front_left.getSpeedInCan();
    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)] = this->_front_right.getSpeedInCan();
    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)] = this->_back_left.getSpeedInCan();
    _data_frame_array.can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)] = this->_back_right.getSpeedInCan();

    esp32_can_pub_.publish(_data_frame_array);
  }

  void JimmBotHardwareInterface::canFeedbackMsgCallback(const jimmbot_msgs::canFrameArray::ConstPtr &feedbackMsgArray)
  {
    std::vector<CanMsgWrapperCommand> update_status_frames
    {
      CanMsgWrapperCommand{this->_front_left, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedbackMsgArray->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_LEFT)]},
      CanMsgWrapperCommand{this->_front_right, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedbackMsgArray->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_FRONT_RIGHT)]},
      CanMsgWrapperCommand{this->_back_left, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedbackMsgArray->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_LEFT)]},
      CanMsgWrapperCommand{this->_back_right, CanMsgWrapperCommand::Command::MOTOR_STATUS_UPDATE, 
                           feedbackMsgArray->can_frames[static_cast<int>(CanMsgWrapper::CanId::COMMAND_WHEEL_BACK_RIGHT)]}
    };

    for (auto& status_frame : update_status_frames)
    {
      status_frame.execute();
    }
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
  ros::init(argc, argv, "jimmbot_base");
  ros::NodeHandle nh, private_nh("~");
  const double _one_second = 1;
  double _control_frequency, _max_wheel_speed;

  private_nh.param<double>("control_frequency", _control_frequency, 10.0);
  private_nh.param<double>("max_wheel_speed", _max_wheel_speed, 2.5);

  jimmbot_base::JimmBotHardwareInterface _jimmbot_base;

  controller_manager::ControllerManager _controller_manager(&_jimmbot_base);
  
  ros::CallbackQueue jimmbot_queue;
  ros::AsyncSpinner jimmbot_spinner(_one_second, &jimmbot_queue);
  
  ros::TimerOptions control_loop_timer(ros::Duration(_one_second / _control_frequency),
                                       boost::bind(jimmbot_base::controlLoopCallback, boost::ref(_jimmbot_base), boost::ref(_controller_manager), _jimmbot_base.getTimeNow()),
                                       &jimmbot_queue);
  ros::Timer control_loop_callback = nh.createTimer(control_loop_timer);

  jimmbot_spinner.start();
  ros::spin();

  return EXIT_SUCCESS;
}