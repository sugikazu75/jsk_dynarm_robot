#pragma once

#include <ros/ros.h>
#include <robstride_actuator_bridge/MotorCommand.h>
#include <sensor_msgs/JointState.h>

class RobstrideBridge
{
public:
  RobstrideBridge(ros::NodeHandle& nh);
  ~RobstrideBridge() = default;

  void jointsControlCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher robstride_joint_command_pub_;  // joint level
  ros::Subscriber joints_control_sub_;          // joint level

  std::map<std::string, int> joint_name_to_motor_index_;
  std::map<std::string, double> joint_name_to_kp_;
  std::map<std::string, double> joint_name_to_kd_;
};
