#include <dynarm/navigation/manipulator_navigator.h>

using namespace aerial_robot_navigation;

ManipulatorNavigator::ManipulatorNavigator() : BaseNavigator()
{
}

void ManipulatorNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
  dynamixel_torque_enable_pub_ = nh_.advertise<spinal::ServoTorqueCmd>("servo/torque_enable", 1);
  robstride_servo_on_pub_ = nh_.advertise<std_msgs::Empty>("robstride_servo_on", 1);
  robstride_servo_off_pub_ = nh_.advertise<std_msgs::Empty>("robstride_servo_off", 1);
}

void ManipulatorNavigator::halt()
{
  BaseNavigator::halt();

  spinal::ServoTorqueCmd gimbal_torque_enable_msg;
  gimbal_torque_enable_msg.index.resize(2 * robot_model_->getRotorNum());  // assume 2 DoF gimbals
  gimbal_torque_enable_msg.torque_enable.resize(2 * robot_model_->getRotorNum());
  std::iota(gimbal_torque_enable_msg.index.begin(), gimbal_torque_enable_msg.index.end(), 0);
  std::fill(gimbal_torque_enable_msg.torque_enable.begin(), gimbal_torque_enable_msg.torque_enable.end(), 0);
  dynamixel_torque_enable_pub_.publish(gimbal_torque_enable_msg);
  ROS_INFO_STREAM("[dynarm][navigation] torque for " << 2 * robot_model_->getRotorNum() << " gimbals disabled");

  robstride_servo_off_pub_.publish(std_msgs::Empty());
  ROS_INFO_STREAM("[dynarm][navigation] servo off the robstride");
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::ManipulatorNavigator, aerial_robot_navigation::BaseNavigator);
