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
  robstride_servo_on_pub_ = nh_.advertise<std_msgs::Empty>("robstride_servo_on", 1);
  robstride_servo_off_pub_ = nh_.advertise<std_msgs::Empty>("robstride_servo_off", 1);
}

void ManipulatorNavigator::halt()
{
  BaseNavigator::halt();

  ROS_INFO_STREAM("[dynarm][navigation] servo off the robstride");
  robstride_servo_off_pub_.publish(std_msgs::Empty());
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::ManipulatorNavigator, aerial_robot_navigation::BaseNavigator);
