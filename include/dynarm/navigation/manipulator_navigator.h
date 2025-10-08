#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <spinal/ServoTorqueCmd.h>
#include <numeric>

namespace aerial_robot_navigation
{
class ManipulatorNavigator : public BaseNavigator
{
public:
  ManipulatorNavigator();
  virtual ~ManipulatorNavigator() = default;

  virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du);

  void halt() override;

private:
  ros::Publisher dynamixel_torque_enable_pub_;
  ros::Publisher robstride_servo_on_pub_;
  ros::Publisher robstride_servo_off_pub_;
};
}  // namespace aerial_robot_navigation
