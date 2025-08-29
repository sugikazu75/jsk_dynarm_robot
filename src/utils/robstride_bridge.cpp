#include <dynarm/utils/robstride_bridge.h>

RobstrideBridge::RobstrideBridge(ros::NodeHandle& nh) : nh_(nh)
{
  robstride_joint_command_pub_ = nh_.advertise<robstride_actuator_bridge::MotorCommand>("robstride_joint_command", 10);
  joints_control_sub_ = nh_.subscribe("joints_ctrl", 1, &RobstrideBridge::jointsControlCallback, this);

  XmlRpc::XmlRpcValue all_servo_params;
  nh.getParam("robstride", all_servo_params);
  for (auto robstride_param : all_servo_params)
  {
    if (robstride_param.first.find("controller") != std::string::npos)
    {
      int index = robstride_param.second["index"];
      std::string name = robstride_param.second["name"];
      double kp = robstride_param.second["kp"];
      double kd = robstride_param.second["kd"];
      joint_name_to_motor_index_[name] = index;
      joint_name_to_kp_[name] = kp;
      joint_name_to_kd_[name] = kd;
    }
  }
}

void RobstrideBridge::jointsControlCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); i++)
  {
    if (joint_name_to_motor_index_.find(msg->name.at(i)) != joint_name_to_motor_index_.end())
    {
      robstride_actuator_bridge::MotorCommand motor_command_msg;
      motor_command_msg.index = joint_name_to_motor_index_.at(msg->name.at(i));
      motor_command_msg.angle = msg->position.at(i);
      motor_command_msg.velocity = msg->velocity.at(i);
      motor_command_msg.torque = msg->effort.at(i);
      motor_command_msg.kp = joint_name_to_kp_.at(msg->name.at(i));
      motor_command_msg.kd = joint_name_to_kd_.at(msg->name.at(i));
      robstride_joint_command_pub_.publish(motor_command_msg);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robstride_bridge");
  ros::NodeHandle nh;

  RobstrideBridge* robstride_bridge = new RobstrideBridge(nh);
  ros::spin();
  delete robstride_bridge;

  return 0;
}
